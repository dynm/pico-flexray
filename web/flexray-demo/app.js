'use strict';
const $ = id => document.getElementById(id);
const esc = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const state = {mode:'frame-gen', step:0, playing:false, elapsed:0, cycle:3, phase:0, host:true, sleeping:false, selected:'p2s3', speed:1};
const connection = (from,to,type='signal') => ({from,to,type});
const step = (tag,title,description,formula,note,active,routes) => ({tag,title,description,formula,note,active,routes});
const C = connection;
const frameGenSteps = [
  step('01 / ACQUIRE','Find the hardware FSS edge','The independent capture qualifies idle, waits for TSS LOW, checks receive-side TXEN, then waits for the FSS rising edge. GPIO10 emits a 15-clock marker; the real streamer supplies the header label afterward.','RXD GP26 → PIO2 SM3\nFSS → GP10 marker → PIO0 SM1','Idle is checked every 6 clocks. WAIT captures TSS/FSS edges; CPU interrupt arrival does not anchor timing.',['p2s3','p0s1','p1s0'],[C('bus','p2s3'),C('p2s3','p0s1'),C('bus','p1s0'),C('p0s1','memory','dma')]),
  step('02 / MEASURE','Measure slot length from ID gaps','SM1 samples one marker bit per system clock into a 4 KiB DMA ring. Software retains the lowest five distinct observed static IDs, accounts for missing slots using ID differences, and estimates slot and cycle periods.','Tslot = ΔFSS / ΔID\nTcycle = (Δt − ΔID × Tslot) / Δcycle','Example candidates: 2, 4, 7, 9, 10. IDs need not be 1..5. The default requires 12 qualifying measurements.',['p0s1','p2s3','p1s0'],[C('p2s3','p0s1'),C('p0s1','memory','dma'),C('memory','cpu','dma'),C('p1s0','cpu','irq')]),
  step('03 / LOCK','Reuse the SM for pacing','After slot and cycle lock, continuous sampling stops. PIO0 SM1 replaces its 1-word sampler with the 16-word pace program; SM3 handles short phase captures. A real FSS starts the hardware countdown, then a header binds ID/cycle.','PIO0 SM1: measure → pace\nPIO2 IRQ next 0 → PIO0 IRQ0','The first beat allows about one cycle. Targets may precede the reference ID. Runtime PIO0 uses 30/32 instruction words.',['p0s1','p0s3','p2s3'],[C('p2s3','p0s1','irq'),C('cpu','memory','dma'),C('memory','p0s1','dma')]),
  step('04 / PREPARE','Prepare C during slot B','In the preceding logical slot, core1 selects a normal or null template and updates only the cycle and precomputed CRC. It preloads the packet FIFO, then publishes a single-use framing waveform. The source remains owned until wire transmission completes.','cycle & 3 = 3 and data available → DATA\notherwise → 18-byte NULL','Change the cycle or turn off host data below. Missing data does not move the FSS deadline. The host need not trigger transmission every 5 ms.',['p0s1','p0s0','p2s2'],[C('host','cpu','dma'),C('cpu','memory','dma'),C('memory','p2s2','dma'),C('memory','p0s0','dma'),C('p0s1','cpu','irq')]),
  step('05 / TRANSMIT','A hardware beat starts the frame','The slot C beat sets IRQ3. The inducer sets PIO1 IRQ7 and drives GP17 LOW before emitting TSS/FSS/BSS framing on GP16. PIO1 SM2 drives TXEN LOW; PIO2 SM2 outputs the actual bytes from its FIFO.','pace IRQ3 → inducer\nGP17 ↓ → TXEN ↓\nGP16 + packet FIFO → GP4','GP16 carries framing timing, not payload bytes. The original injector instructions are unchanged. Frame generation adds no local RX streamer.',['p0s1','p0s0','p1s2','p2s2'],[C('p0s1','p0s0','irq'),C('p0s0','p1s2','irq'),C('p0s0','p2s2'),C('p2s2','output'),C('p1s2','output')]),
  step('06 / DONE','Release output after FES and idle','After FES, TXEN ownership continues through 11 idle bits. The inducer then releases GP17/IRQ7 and emits an independent DONE. Core1 releases packet C and prepares D during the remainder of slot C.','post-FES idle = 165 clocks\nPIO0 IRQ2 → frame_gen_done_irq()','Real RX frame-end callbacks handle received traffic only. An ACTIVE frame generator does not turn a real RX event into local completion.',['p0s0','p1s2','p2s2'],[C('p0s0','p1s2','irq'),C('p0s0','cpu','irq'),C('cpu','memory','dma'),C('memory','p2s2','dma')]),
  step('07 / PHASE','Capture phase in a bounded window','The view below zooms into a reference slot. PIO0 SM3 samples after pace IRQ4. The first 256 clocks must contain exactly one complete marker. The real header must be processed and validated before the finite DMA deadline.','expected = TSSclocks − 19 − phase\nerror = marker_position − expected','TSS8 defaults to expected position 101. Move the phase slider to shift the real FSS relative to prediction; within-cycle slot lengths remain fixed.',['p2s3','p0s3','p0s1','p1s0'],[C('p0s1','p0s3','irq'),C('p2s3','p0s3'),C('p0s3','memory','dma'),C('p1s0','cpu','irq')]),
  step('08 / CORRECT','Adjust only the cycle tail','The CPU updates only periods[gap_index]. Sixty-four guard descriptors span one slot and keep the mutable tail beyond DMA/FIFO prefetch. The tail produces no slot pace events.','tail = nominal_tail + error\nstatic slot interval = Tslot','Positive error lengthens the tail; negative error shortens it, bounded by tolerance and remaining tail time. This does not adjust every slot interval as a PLL would.',['p0s1'],[C('cpu','memory','dma'),C('memory','p0s1','dma')]),
  step('09 / TRACK','Consume each correction once','After pace DMA consumes the tail, reset DMA restores its nominal value and reload DMA restarts the schedule. Bounded calibration continues each cycle. Brief reference loss retains prediction; about three cycles without valid calibration pause output and restart acquisition.','pace DMA → reset DMA → reload\nNext cycle: measure phase again','Simulate sleep below. This animation returns to preparation after the last step. Real phase calibration occurs in the reference slot, not necessarily after C/D.',['p0s1','p0s3','p2s3'],[C('memory','p0s1','dma'),C('p2s3','p0s3'),C('p0s3','memory','dma')])
];
function mitmSteps(dual) {
  const rx = dual?'p0s1':'p1s1', tx = dual?'p2s2':'p2s0';
  const destination = dual?'FR3':'FR1';
  return [
    step('01 / FORWARD',dual?'Two independent bridge pairs':'Forward the real signal',dual?'PIO1 receives FR1/FR2 and PIO0 receives FR3/FR4. Four PIO2 SMs provide the four output directions. Real header/frame-end events combine source tags for host identification.':'This example follows FR2 → FR1. PIO2 SM0 forwards real RXD levels onto TXD. PIO1 SM1 decodes bytes and drives FR1 TXEN; RX DMA writes the receive ring.',dual?'FR1 ↔ FR2 / FR3 ↔ FR4\n4 RX SM + 4 forwarder SM':'GP6 → PIO2 SM0 → GP28\nPIO1 SM1 → TXEN GP27','Real bus edges drive forwarding timing. No local slot pace or per-frame host data is required.',dual?['p1s0','p1s1','p0s0','p0s1','p2s0','p2s1','p2s2','p2s3']:['p1s1','p2s0'],dual?[C('bus','p1s0'),C('bus','p0s1'),C('bus','p2s1'),C('bus','p2s2'),C('p2s1','output'),C('p2s2','output')]:[C('bus','p1s1'),C('bus','p2s0'),C('p1s1','memory','dma'),C('p2s0','output')]),
    step('02 / HEADER','A real header authorizes preparation','When the primary bridge sees FID6 with cycle&3=2, its real header IRQ checks the cached FID8 template and single-use override. Only the first four payload bytes are replaced; cycle and frame CRC are updated.',`trigger FID6 / base2 → target FID8\n${dual?'OFF: destination FR3':'ON: destination FR1'}`,'This assumes a valid cached template and an unexpired host override. In dual bridge mode, FR3/4 headers identify sources; the primary bridge header still prepares the current rule.',['p1s1'],[C('host','cpu','dma'),C('p1s1','cpu','irq'),C('memory','cpu','dma')]),
    step('03 / ARM','A real frame end arms DMA',`The real FID6 frame-end callback rechecks command ownership and expiry, then hands the prepared template to the ${destination} injector DMA. Data enters the FIFO and waits for real target edges; the interrupt does not generate TSS.`,`PIO1 flag3 → inject_prepared_frame\npacket DMA → ${dual?'PIO2 SM2':'PIO2 SM0'}`,'A new header discards stale prepared state. Turn off host data to see that no replacement is armed without a fresh override.',[tx,'p1s1'],[C('p1s1','cpu','irq'),C('memory',tx,'dma')]),
    step('04 / INJECT',`Follow real FID8 toward ${destination}`,`At the real target TSS/FSS, the forwarder SM sees a nonempty FIFO and follows the original injector path. The real streamer controls TXEN and IRQ7 echo exclusion.${dual?' Here the target enters FR4, is received by PIO0 SM1, and exits FR3 through PIO2 SM2.':''}`,`Real RXD → TSS / FSS\nFIFO nonempty → replace; empty → forward`,'MITM does not create missing frames. Without a real target there is no target TSS. Without a valid override, normal forwarding continues.',[rx,tx],[C('bus',rx),C('bus',tx),C(tx,'output'),C(rx,'output')]),
    step('05 / RELEASE','Release TXEN and await the next frame','On receive completion, the streamer emits frame-end, releases IRQ7, and returns to idle. The override has been consumed. If host data stops or expires, normal forwarding continues; another replacement requires fresh valid data.',dual?'PIO0 / PIO1 each own IRQ7\nBoth bridge pairs return to idle':'PIO1 frame-end → release IRQ7\nNormal forwarding continues','Missing MITM data leaves forwarding unchanged. Missing data for enabled, synchronized frame generation sends null in the reserved slot. These policies intentionally differ.',[rx,tx],[C(rx,'cpu','irq'),C(tx,'output')])
  ];
}
const sleepSteps = [
  step('SLEEP / HOLDOVER','Brief holdover without a reference','The bus sleeps HIGH with no new valid FSS calibration. Pace briefly follows the learned cycle period. Host data is still consumed only once under the normal rules.','No valid calibration < 3 × cycle\nRetain prediction','RX no longer supplies FSS markers to capture. This illustrates the watchdog boundary of about three cycles.',['p0s1'],[C('memory','p0s1','dma')]),
  step('SLEEP / LOST','Pause output and invalidate stale data','After about three learned cycles without valid calibration, new pace events and triggers stop. Stale payloads are discarded and the synchronization generation increments. Active frames finish through independent DONE; even null frames pause without trustworthy slots.','sync_losses +1 / reason1\nenabled retained · resyncing = 1','Null transmission pauses on the sleeping bus. The host does not need to enable again.',[ 'p0s0'],[C('p0s0','cpu','irq')]),
  step('SLEEP / REACQUIRE','Reacquire when the bus returns','The simulated bus returns. The original sampling SM/DMA restarts acquisition, clears candidates, and selects the lowest five observed IDs again. Lock and valid phase calibration restore the same static rules.','Discard old period → reacquire\nStale payloads do not survive recovery','The next step returns to acquisition. Recovery requires valid phase calibration; without fresh payloads, output resumes with null frames.',['p2s3','p0s1','p1s0'],[C('bus','p2s3'),C('p2s3','p0s1'),C('p0s1','memory','dma')])
];
function stages(){return state.sleeping?sleepSteps:state.mode==='frame-gen'?frameGenSteps:mitmSteps(state.mode==='dual');}
function current(){return stages()[state.step];}
function role(name,sub,io,dma,irq,note,status='allocated'){return {name,sub,io,dma,irq,note,status};}
function roles(){
  const dual=state.mode==='dual', acquisition=state.mode==='single'||(state.sleeping?state.step===2:state.step<2);
  const r={};
  const rx=(from,to,pins,bank)=>role(`RX · FR${from}`,`streamer → FR${to} TXEN`,pins,'RX FIFO → ring buffer',`${bank} flag4 header / flag3 end / flag7 echo`,'The real receive program handles RX bytes and TXEN. Header and frame-end callbacks still belong to real traffic.');
  const tx=(from,to,pins)=>role(`TX · FR${to}`,`forward / inject · FR${from} → FR${to}`,pins,'Original MITM packet DMA → TX FIFO','Started by real RX edges, not frame generation pace','All four directions share the original 22-word forwarder/injector program.');
  r.p0s0=dual?rx(3,4,'GP8 RXD → GP22 TXEN','PIO0'):role('Inducer','GP16 framing / GP17 ownership','OUT GP16; side-set GP17','Finite pulse DMA → TX FIFO','PIO0 IRQ3 input; PIO1 IRQ7; PIO0 DONE IRQ2','12 words. No local receive streamer. Descriptors include the full 11-bit post-FES idle.');
  r.p0s1=dual?rx(4,3,'GP21 RXD → GP9 TXEN','PIO0'):acquisition?role('Measure','1 clock / bit · FSS sampling','IN GP10 marker','Sample DMA → 4 KiB ring','Intervals measured from hardware marker positions','1 word during acquisition. After lock, continuous sampling stops and this SM switches to the 16-word pace program.'):role('Slot pace','Fixed slots / adjustable cycle tail','Initial PIO0 IRQ0 from capture','3 DMA channels: pace / reset / reload','Outputs: IRQ3 inducer, IRQ4 phase, IRQ5 CPU','16 words. Every static slot has a beat; the tail has none. Calibration changes the tail once.');
  r.p0s2=role('Free','Unused','—','—','—','This SM is unused by the demo firmware.','free');
  r.p0s3=dual?role('Free','Unused','—','—','—','Four-channel mode has no phase program.','free'):acquisition?role('Phase · waiting','Reserved; inactive during acquisition','Reads GP10 after lock','Reuses sample DMA after lock','Waits for PIO0 IRQ4 after lock','The 2-word phase program is not loaded during acquisition.','reserved'):role('Phase capture','Bounded window / 256-clock validation','IN GP10 marker','Reuses acquisition sample DMA','WAIT PIO0 IRQ4','2 words. Tail corrections require an active DMA window. Expired results cannot affect later cycles.');
  r.p1s0=rx(1,2,'GP26 RXD → GP5 TXEN','PIO1');r.p1s1=rx(2,1,'GP6 RXD → GP27 TXEN','PIO1');
  r.p1s2=dual?role('Free','Unused','—','—','—','OFF compiles out frame generation TXEN control.','free'):role('TXEN edge','GP17 → FR2 TXEN','WAIT GP17; SET GP5','No DMA','No RX IRQ','4 words: WAIT/SET/WAIT/SET. WAIT does not write TXEN, so real forwarding can control the same pin between local frames.');
  r.p1s3=role('Free','SM available','—','—','—',dual?'PIO1 uses 28/32 instruction words.':'PIO1 uses all 32 instruction words. An unused SM does not imply room for another program.','free');
  r.p2s0=tx(2,1,'GP6 RXD → GP28 TXD');r.p2s1=tx(1,2,'GP26 RXD → GP4 TXD');
  r.p2s2=dual?tx(4,3,'GP21 RXD → GP10 TXD'):role('Local injector','Original program reused · FR2 frame generation','IN GP16; OUT GP4','Finite packet DMA → TX FIFO','Induced by GP16 TSS/FSS','Shares the original 22-word program, entering at offset 3, WAIT TSS. Actual bytes come from the FIFO.');
  r.p2s3=dual?tx(3,4,'GP8 RXD → GP16 TXD'):role('FSS capture','idle → WAIT TSS / FSS','IN GP26 + GP27 TXEN; OUT GP10','No direct DMA; RX token to CPU','IRQ next 0 → PIO0 bootstrap','10 words. GP10 markers span 15 clocks; idle is checked every 6 clocks. The real streamer idle loop is unchanged.');
  return r;
}
function instructionUse(){if(state.mode==='dual')return [[28],[28],[22]];return [(roles().p0s1.name==='Measure')?[12,1]:[12,16,2],[28,4],[22,10]];}
function renderBanks(){
 const data=roles(), use=instructionUse(), active=current().active;
 $('pio-grid').innerHTML=[0,1,2].map(p=>`<section class="pio-bank"><div class="bank-header"><b>PIO ${p}</b><span>${use[p].reduce((a,b)=>a+b,0)} / 32 W</span></div><div class="word-meter">${use[p].map(n=>`<i style="width:${n/32*100}%"></i>`).join('')}</div>${[0,1,2,3].map(s=>{const id=`p${p}s${s}`,r=data[id];return `<button id="${id}" class="sm ${active.includes(id)?'active':''} ${r.status==='free'?'free':''} ${state.selected===id?'inspected':''}" data-sm="${id}" aria-label="PIO${p} SM${s} ${esc(r.name)}" aria-pressed="${state.selected===id}"><span class="sm-top">SM ${s}<i class="indicator"></i></span><span class="sm-title">${esc(r.name)}</span><small>${esc(r.sub)}</small></button>`}).join('')}</section>`).join('');
 $('words').textContent=use.map(a=>a.reduce((x,y)=>x+y,0)+'/32').join(' · ');
 $('dma-count').textContent=state.mode==='dual'?'8':'10';$('active-count').textContent=active.length+' / 12';$('firmware').textContent='STATIC_TX '+(state.mode==='dual'?'OFF':'ON');
 $('allocation-note').textContent=state.mode==='dual'?'Four-channel firmware compiles out frame generation. PIO0 receives the second bridge pair. Highlights indicate this step, not that other channels have stopped.':state.mode==='single'?'Single bridge mode uses ON firmware with frame generation disabled. Capture and sampling may acquire in the background; resources remain reserved. Highlights follow only MITM.':'ON firmware: highlights show this step; other forwarding resources remain allocated. PIO0 changes programs between acquisition and runtime.';
 $('bus-title').textContent=state.mode==='dual'?'Two FlexRay bridge pairs':'FlexRay bus';$('bus-detail').textContent=state.mode==='dual'?'FR1/2 + FR3/4 · real RXD':state.mode==='single'?'Example FR2 → FR1 · RXD GP6':'RXD GP26 · hardware FSS';
 $('output-title').textContent=state.mode==='dual'?'FR1..FR4 outputs':state.mode==='single'?'FR1 output':'FR2 output';$('output-detail').textContent=state.mode==='dual'?'TXD GP28 / 4 / 10 / 16':state.mode==='single'?'TXD GP28 · TXEN GP27':'TXD GP4 · TXEN GP5';
 $('host-detail').textContent=state.mode==='frame-gen'?'0x94 payload · 0x95 enable':'0x90 override · 0x91 enable';
 renderInspector();renderWireDetails();requestAnimationFrame(drawConnections);
}
function renderInspector(){const r=roles()[state.selected];$('inspect-title').textContent=state.selected.replace(/p(\d)s(\d)/,'PIO$1 · SM$2')+' / '+r.name;$('inspect-body').innerHTML=`<div class="inspector-grid"><div><small>GPIO / signal</small><p>${esc(r.io)}</p></div><div><small>FIFO / DMA</small><p>${esc(r.dma)}</p></div><div><small>IRQ / trigger</small><p>${esc(r.irq)}</p></div></div><p class="inspector-note">${esc(r.note)}</p>`;}
function renderWireDetails(){
 const labels = {
  'p2s3:p0s1': current().routes.some(r=>r.from==='p2s3'&&r.to==='p0s1'&&r.type==='irq')?'PIO2 → PIO0 · IRQ0':'GP10 · FSS marker',
  'p2s3:p0s3':'GP10 · FSS marker', 'p0s1:p0s0':'PIO0 IRQ3 · pace → inducer',
  'p0s1:p0s3':'PIO0 IRQ4 · phase start', 'p0s1:cpu':'PIO0 IRQ5 · pace ISR',
  'p0s0:p1s2':'GP17 → TXEN SM / IRQ7 → PIO1', 'p0s0:p2s2':'GP16 · TSS / FSS / BSS',
  'p0s0:cpu':'PIO0 IRQ2 · independent DONE', 'p1s0:cpu':'PIO1 · real RX IRQ',
  'p1s1:cpu':'PIO1 · real RX IRQ', 'p0s1:memory':'sample DMA → ring',
  'p0s3:memory':'finite phase DMA → ring', 'memory:p0s1':'periods → pace DMA',
  'memory:p0s0':'pulse DMA → FIFO', 'memory:p2s2':state.mode==='dual'?'MITM DMA → FR3 FIFO':'packet DMA → local FIFO',
  'memory:p2s0':'MITM DMA → FR1 FIFO', 'p1s2:output':'GP5 · FR2 TXEN',
  'p2s2:output':state.mode==='dual'?'GP10 · FR3 TXD':'GP4 · FR2 TXD',
  'p2s0:output':'GP28 · FR1 TXD', 'p2s1:output':'GP4 · FR2 TXD',
  'host:cpu':state.mode==='frame-gen'?'USB / NCM · 0x94 payload':'USB / NCM · MITM override'
 };
 $('wire-details').innerHTML=current().routes.filter(r=>labels[r.from+':'+r.to]&&(state.host||r.from!=='host')).map(r=>`<span class="${r.type}">${esc(labels[r.from+':'+r.to])}</span>`).join('');
}
function drawConnections(){
 const parent=$('topology').getBoundingClientRect();const svg=$('connections');svg.setAttribute('viewBox',`0 0 ${parent.width} ${parent.height}`);
 const center=id=>{const r=$(id).getBoundingClientRect();return {x:r.x-parent.x,y:r.y-parent.y,w:r.width,h:r.height}};
 let routes=current().routes;
 if(!state.host)routes=routes.filter(r=>r.from!=='host'&&!(state.mode!=='frame-gen'&&state.step===2&&r.type==='dma'));
 svg.innerHTML=routes.map((r,i)=>{const a=center(r.from),b=center(r.to);let x1=a.x+a.w/2,y1=a.y+a.h/2,x2=b.x+b.w/2,y2=b.y+b.h/2;
 if(Math.abs(x1-x2)>Math.abs(y1-y2)){const sign=x2>x1?1:-1;x1+=sign*a.w/2;x2-=sign*b.w/2;}else{const sign=y2>y1?1:-1;y1+=sign*a.h/2;y2-=sign*b.h/2;}
 const d=`M${x1},${y1} C${x1},${(y1+y2)/2} ${x2},${(y1+y2)/2} ${x2},${y2}`;
 return `<path id="route-${i}" class="route ${r.type}" d="${d}"/><circle class="flow-dot ${r.type}" r="3"><animateMotion dur="${2/state.speed}s" repeatCount="indefinite" path="${d}"/></circle>`;
 }).join('');if(!state.playing)svg.pauseAnimations();else svg.unpauseAnimations();
}
const text=(x,y,t,cls='wave-label')=>`<text x="${x}" y="${y}" class="${cls}">${esc(t)}</text>`;
const line=(x1,y1,x2,y2,cls='wave-axis')=>`<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" class="${cls}"/>`;
const path=(d,cls='wave')=>`<path d="${d}" class="${cls}"/>`;
function frameKind(){return state.host&&(state.cycle&3)===3?'DATA':'NULL';}
function scope(){
 let content='',note='',title='',chip='Illustration · not to scale';
 if(state.sleeping){title='Sleep and resynchronization';const phase=state.step;content=text(5,32,'RXD')+path('M80 24 H620')+text(5,83,'PACE');for(let i=0;i<9;i++){let x=85+i*58;content+=path(`M${x} 90 v-20 h5 v20 h43`,i<4||phase===0?'wave blue':'wave dashed');}content+=text(92,127,'Holdover', 'wave-text')+text(315,127,'~3 cycles → pause','scope-tag')+text(440,166,phase===2?'Bus returns → reacquire':'enable retained / stale payload cleared','wave-small');note='Active frames finish before TXEN is released. Once predicted output stops, wait for the real bus to return.';
 }else if(state.mode!=='frame-gen'){
 title=state.host?'Real trigger → single replacement':'Host stops → forward unchanged';chip='FID6 → FID8 · cycle&3 = 2';
 content=text(5,30,'RXD')+path('M80 20 H110 V38 H143 V20 H278 V38 H305 V20 H560 V38 H588 V20 H620')+text(152,15,'FID6 · trigger','wave-small')+text(355,15,'FID8 · target','wave-small');
 content+=text(5,78,'CPU')+line(80,82,620,82)+line(194,65,194,94,'marker-line')+line(278,65,278,94,'marker-line')+text(160,61,'header','wave-small')+text(244,106,'frame-end','wave-small');
 content+=text(5,142,'TXD')+path('M80 132 H110 V150 H143 V132 H278 V150 H305 V132 H620','wave blue');
 content+=`<rect x="322" y="121" width="212" height="33" rx="4" class="slot-rect ${state.host?'target':'null'}"/>`+text(348,142,state.host?'cached frame + override':'original frame',state.host?'wave-text':'wave-small');
 content+=text(80,185,state.host?'header prepares · frame-end arms DMA · real target FSS starts TX':'No new override in FIFO → original forwarder follows real levels','wave-small');note='Intervening frames such as FID7 are omitted. The header ISR does not immediately transmit the target; missing data does not generate null frames here.';
 }else if(state.step<2){
 title='Measure FSS intervals, including empty slots';chip='150 MHz sampling · example candidates';
 content=text(5,40,'GP10');const ids=[2,4,7,9,10];let d='M80 49';ids.forEach(id=>{const x=90+(id-2)*58;d+=` H${x} V29 H${x+6} V49`;content+=text(x-10,19,'ID'+id,'wave-small');});d+=' H620';content+=path(d);
 content+=line(95,90,210,90,'marker-line')+text(115,80,'ΔID = 2','wave-text')+line(210,120,384,120,'marker-line')+text(248,111,'ΔID = 3','wave-text');
 content+=text(82,161,'Tslot = (FSS₂ − FSS₁) / (ID₂ − ID₁)','wave-text')+text(82,186,'Absent IDs 3, 5, 6, 8 still occupy slot time','wave-small');note='Only five candidate IDs are shown. Lock requires repeated measurements, median filtering, and tolerance checks, not just five frames.';
 }else if(state.step===4||state.step===5){
 title='TXEN covers the complete local frame';chip='TSS8 · payload18 · framing detail';
 content=text(5,30,'GP16')+path('M80 22 H115 V42 H190 V22 H220 V42 H385 V22 H397 V42 H460 V22 H620');
 content+=text(5,88,'TXD GP4')+path('M80 80 H118 V100 H193 V80 H226 V100 H239 V80 H254 V100 H275 V80 H299 V100 H318 V80 H344 V100 H369 V80 H401 V100 H460 V80 H620','wave blue');
 content+=text(5,145,'TXEN')+path('M80 132 H113 V156 H562 V132 H620','wave amber');
 content+=text(132,65,'TSS','wave-small')+text(235,65,'Actual packet bytes','wave-small')+text(422,65,'FES','wave-small')+text(478,118,'11-bit idle','scope-tag')+line(562,14,562,173,'marker-line')+text(549,190,'DONE','wave-text');
 note='GP16 provides framing; GP4 outputs real data. GP17 ownership / IRQ7 is acquired before TSS and released after post-FES idle. Bits and durations are illustrative, not to scale.';
 }else if(state.step>=6){
 title=state.step===7?'Correct the tail; keep slots fixed':'Bounded phase capture in the reference slot';chip='Default expected = 101 clocks';
 const scale=540/256, x=80+101*scale, actual=x+state.phase*scale;
 content=`<rect x="80" y="8" width="540" height="97" fill="#193128" opacity=".45"/>`+text(488,20,'256-clock marker window','wave-small')+text(5,35,'Predicted FSS')+path(`M80 43 H${x} V24 H${x+8} V43 H620`,'wave dashed');
 content+=text(5,84,'Observed GP10')+path(`M80 91 H${actual} V70 H${actual+15*scale} V91 H620`)+line(x,13,x,108,'marker-line')+line(actual,63,actual,108,'marker-line')+text(400,76,`${state.phase>=0?'+':''}${state.phase} clocks`,'wave-text');
 for(let i=0;i<5;i++)content+=`<rect x="${80+i*62}" y="129" width="55" height="26" rx="3" class="slot-rect"/>`+text(89+i*62,147,'slot','wave-small');
 content+=`<rect x="399" y="129" width="${160+state.phase*1.5}" height="26" rx="3" class="slot-rect target"/>`+text(419,147,`tail ${state.phase>=0?'+':''}${state.phase}`,'wave-text')+text(80,186,'Tslot unchanged', 'wave-small')+text(395,186,'Consumed tail resets to nominal','wave-small');
 note='Calibration adjusts the next cycle boundary, not frames already sent. Positive error lengthens the tail; negative error shortens it. Invalid or expired markers leave the tail unchanged.';
 }else{
 title='A beat per static slot; local frames only in C/D';chip=`cycle ${String(state.cycle).padStart(2,'0')} · ${frameKind()}`;
 for(let i=0;i<16;i++){const x=80+i*28;const target=i===11||i===12;content+=`<rect x="${x}" y="37" width="24" height="50" rx="3" class="slot-rect ${target?(frameKind()==='DATA'?'target':'null'):''}"/>`+text(x+6,59,(i+1).toString(16).toUpperCase(),target?'wave-text':'wave-small');if(target)content+=text(x+5,76,frameKind()==='DATA'?'D':'N','wave-small');content+=path(`M${x} 135 v-17 h3 v17 h21`,'wave blue');}
 content+=text(5,59,'SLOT')+text(5,130,'PACE')+`<rect x="535" y="37" width="85" height="50" rx="3" class="slot-rect"/>`+text(550,65,'TAIL','wave-small')+line(535,135,620,135)+text(410,180,'B prepares C · C DONE prepares D','wave-small');note=`C/D rep4 base3; currently ${(state.cycle&3)===3?'at base3':'outside base3'}, host ${state.host?'streaming':'stopped'} → ${frameKind()==='DATA'?'one normal payload consumed per target':'null with 18 zero payload bytes'}. The tail emits no pace.`;
 }
 $('timeline-title').textContent=title;$('timeline-chip').textContent=chip;$('timeline-note').textContent=note;$('timeline').innerHTML=`<svg viewBox="0 0 640 205" role="img" aria-label="${esc(title)}"><title>${esc(title)}</title>${content}<line id="scope-cursor" class="scope-cursor" x1="80" x2="80" y1="5" y2="194"/></svg>`;
 $('cycle-control').hidden=state.mode!=='frame-gen';
}
function render(){
 const s=current();document.querySelectorAll('[data-mode]').forEach(b=>{b.classList.toggle('selected',b.dataset.mode===state.mode);b.setAttribute('aria-pressed',String(b.dataset.mode===state.mode));});
 $('steps').innerHTML=stages().map((s,i)=>`<button class="step-button ${i===state.step?'current':i<state.step?'past':''}" data-step="${i}" title="${esc(s.title)}" aria-label="Step ${i+1} ${esc(s.title)}" aria-current="${i===state.step?'step':'false'}">${i+1}</button>`).join('');
 $('step-counter').textContent=`${String(state.step+1).padStart(2,'0')} / ${String(stages().length).padStart(2,'0')}`;
 $('step-tag').textContent=s.tag;$('step-title').textContent=s.title;$('step-description').textContent=s.description;$('formula').textContent=s.formula;$('key-point').textContent=s.note;
 $('frame-gen-controls').hidden=state.mode!=='frame-gen';$('mitm-controls').hidden=state.mode==='frame-gen';$('experiment-title').textContent=state.mode==='frame-gen'?'Explore phase changes':'Follow real frames; replace on request';
 $('mitm-direction').textContent=state.mode==='dual'?'OFF rule: primary FID6/base2 triggers FID8 injection toward FR3. The two bridges do not independently generate arbitrary FID8 frames.':'ON rule: FID6/base2 triggers FID8 injection toward FR1. Single bridge mode is one bidirectional pair, not one-way forwarding.';
 $('phase-value').textContent=(state.phase>=0?'+':'')+state.phase+' clk';$('phase-comparison').innerHTML=`Predicted marker <strong>101</strong> → observed <strong>${101+state.phase}</strong><br>Tail <strong>nominal ${state.phase>=0?'+':'−'} ${Math.abs(state.phase)}</strong> · static slot intervals unchanged`;
 $('cycle-value').textContent=String(state.cycle).padStart(2,'0');$('cycle').value=state.cycle;$('sleep').textContent=state.sleeping?'Return to normal demo':'Simulate bus sleep';
 renderBanks();scope();renderProgress();document.dispatchEvent(new Event("demo:change"));
}
function renderProgress(){ const cursor=$('scope-cursor');if(cursor){const x=80+state.elapsed/6000*540;cursor.setAttribute('x1',x);cursor.setAttribute('x2',x);cursor.style.opacity=state.playing?'.55':'0';} $('play').textContent=state.playing?'Ⅱ Pause':'▶ Play';$('progress').style.width=((state.step+state.elapsed/6000)/stages().length*100)+'%';$('progress-label').textContent=state.sleeping?'Sleep → pause → reacquire':state.mode==='frame-gen'?'Capture → acquire → hardware pace → transmit → bounded phase correction':'Forward → header prepares → frame-end authorizes → replace target';}
function go(index){state.step=Math.max(0,Math.min(stages().length-1,index));state.elapsed=0;render();}
function advance(){if(state.step<stages().length-1)go(state.step+1);else if(state.sleeping){state.sleeping=false;go(0);}else if(state.mode==='frame-gen'){state.cycle=(state.cycle+1)&63;go(3);}else go(0);}
function setMode(mode){state.mode=mode;state.sleeping=false;state.step=0;state.elapsed=0;state.playing=false;state.selected=mode==='frame-gen'?'p2s3':mode==='dual'?'p0s1':'p1s1';render();}
$('pio-grid').addEventListener('click',e=>{const b=e.target.closest('[data-sm]');if(!b)return;state.selected=b.dataset.sm;document.querySelectorAll('.sm').forEach(x=>{x.classList.toggle('inspected',x.id===state.selected);x.setAttribute('aria-pressed',String(x.id===state.selected));});renderInspector();document.dispatchEvent(new Event('demo:inspect'));});
document.querySelectorAll('[data-mode]').forEach(b=>b.addEventListener('click',()=>setMode(b.dataset.mode)));
$('steps').addEventListener('click',e=>{const b=e.target.closest('[data-step]');if(b)go(Number(b.dataset.step));});
$('play').addEventListener('click',()=>{state.playing=!state.playing;renderProgress();const svg=$('connections');if(state.playing)svg.unpauseAnimations();else svg.pauseAnimations();});
$('next').addEventListener('click',advance);$('prev').addEventListener('click',()=>go(state.step-1));
$('reset').addEventListener('click',()=>{state.sleeping=false;state.playing=false;state.cycle=3;state.phase=0;state.host=true;$('host-on').checked=true;$('phase').value=0;go(0);});
$('speed').addEventListener('change',e=>{state.speed=Number(e.target.value);drawConnections();});
$('host-on').addEventListener('change',e=>{state.host=e.target.checked;render();});
$('cycle').addEventListener('input',e=>{state.cycle=Number(e.target.value);render();});
$('phase').addEventListener('input',e=>{state.phase=Number(e.target.value);if(!state.sleeping&&state.step<6)state.step=6;state.elapsed=0;render();});
$('drift').addEventListener('click',()=>{state.sleeping=false;go(6);});
$('sleep').addEventListener('click',()=>{state.sleeping=!state.sleeping;go(0);});
let last=0;function tick(t){if(!last)last=t;const dt=Math.min(t-last,100);last=t;if(state.playing&&!document.hidden){state.elapsed+=dt*state.speed;if(state.elapsed>=6000)advance();renderProgress();}requestAnimationFrame(tick);}render();requestAnimationFrame(tick);
new ResizeObserver(()=>requestAnimationFrame(drawConnections)).observe($('topology'));
