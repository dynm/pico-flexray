#!/usr/bin/env bash
# Deploy web/ (signal_gen.html) to Cloudflare Pages.
# Prereqs: npx wrangler whoami (or CLOUDFLARE_API_TOKEN set)
set -e
cd "$(dirname "$0")/.."
cp web/signal_gen.html web/index.html
npx wrangler pages deploy web --project-name=pico-flexray-signal-gen --commit-dirty=true
