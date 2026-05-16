#!/usr/bin/env bash
# Deploy the static web app to Cloudflare Pages.
# Prereqs: npx wrangler whoami (or CLOUDFLARE_API_TOKEN set)
set -euo pipefail

cd "$(dirname "$0")/.."

test -f web/signal_gen.html
test -f web/favicon.svg

cp web/signal_gen.html web/index.html
npx wrangler pages deploy web --project-name=pico-flexray-signal-gen --commit-dirty=true
