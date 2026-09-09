#!/usr/bin/env node
// Build a waveform export OUTSIDE the browser, with the same src/waveExport.js the app uses.
//
// Reads an engine dump on stdin (or from a file argument) and writes the export to stdout:
//
//   { topology, magnetics, analyticalWaveforms, ngspiceOps?, componentWaves?, opIdx?, periods? }
//
// where `magnetics` is api::topology_waveforms' array, `analyticalWaveforms` is design_tas_full's
// map, and `componentWaves` is api::component_waveforms' object — i.e. exactly what the app holds.
//
//   node web/scripts/buildExportJson.mjs dump.json          → the PEAS/MAS operating-point JSON
//   node web/scripts/buildExportJson.mjs --csv dump.json     → the wide CSV
//
// It exists so a schema gate (tests/test_export_schema.py) can validate what the download button
// produces without driving a browser — the module under test is the same file either way.
import { readFileSync } from 'fs'
import { designExcitationsJson, designSignals, toCsv } from '../src/waveExport.js'

const args = process.argv.slice(2)
const csv = args.includes('--csv')
const file = args.find((a) => !a.startsWith('--'))
const ctx = JSON.parse(file ? readFileSync(file, 'utf8') : readFileSync(0, 'utf8'))
ctx.periods ??= 1
ctx.opIdx ??= 0

process.stdout.write(csv
  ? toCsv(designSignals(ctx), { topology: ctx.topology, scope: 'the whole design', periods: ctx.periods })
  : JSON.stringify(designExcitationsJson(ctx), null, 1) + '\n')
