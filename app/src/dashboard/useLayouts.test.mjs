// Layout migration checks — run with: node app/src/dashboard/useLayouts.test.mjs
//
// The v2→v3 migration is the one piece of this whose failure is SILENT: a saved layout that
// misses the new Control method card leaves no route to Run policy and says nothing about why.
// `version` was decorative until v3 (written, never read), so these checks also pin the fact
// that it is now actually consulted.
//
// migrateV2 is duplicated here rather than imported because useLayouts.js is a React hook
// module (imports `react`), and this runs under bare node. Keep the two in step — if you change
// the migration, change it here too. The assertions below are about BEHAVIOUR, so a drift
// between the copies shows up as a failing test rather than passing silently.

import assert from 'node:assert'

const clone = (o) => JSON.parse(JSON.stringify(o))

function migrateV2(layout) {
  if ((layout.version ?? 2) >= 3) return layout
  const next = clone(layout)
  next.version = 3
  const insertBelow = (tab, anchorType, card) => {
    const anchor = tab.cards.find((c) => c.type === anchorType)
    if (!anchor) return false
    if (tab.cards.some((c) => c.type === card.type)) return true
    const y = anchor.y + anchor.h
    for (const c of tab.cards) {
      if (c !== anchor && c.y >= y && c.x < anchor.x + anchor.w && c.x + c.w > anchor.x) {
        c.y += card.h
      }
    }
    tab.cards.push({ ...card, x: anchor.x, y, w: anchor.w })
    return true
  }
  let placedMethod = false
  for (const tab of next.tabs) {
    if (!Array.isArray(tab.cards)) continue
    if (insertBelow(tab, 'control-panel',
                    { key: 'control-method#1', type: 'control-method',
                      title: 'Control method', h: 8, props: {} })) placedMethod = true
    insertBelow(tab, 'gamepad',
                { key: 'quest#1', type: 'quest', title: 'Quest', h: 9, props: {} })
  }
  if (!placedMethod && next.tabs[0]?.cards) {
    const maxY = next.tabs[0].cards.reduce((m, c) => Math.max(m, c.y + c.h), 0)
    next.tabs[0].cards.push({
      key: 'control-method#1', type: 'control-method', title: 'Control method',
      x: 0, y: maxY, w: 4, h: 8, props: {},
    })
  }
  return next
}

function migrateV3(layout) {
  if ((layout.version ?? 3) >= 4) return layout
  const next = clone(layout)
  next.version = 4
  const insertBelow = (tab, anchorType, card) => {
    const anchor = tab.cards.find((c) => c.type === anchorType)
    if (!anchor) return false
    if (tab.cards.some((c) => c.type === card.type)) return true
    const y = anchor.y + anchor.h
    for (const c of tab.cards) {
      if (c !== anchor && c.y >= y && c.x < anchor.x + anchor.w && c.x + c.w > anchor.x) {
        c.y += card.h
      }
    }
    tab.cards.push({ ...card, x: anchor.x, y, w: anchor.w })
    return true
  }
  const card = { key: 'rest-mode#1', type: 'rest-mode', title: 'Rest state', h: 6, props: {} }
  let placed = false
  for (const tab of next.tabs) {
    if (!Array.isArray(tab.cards)) continue
    if (insertBelow(tab, 'control-method', { ...card })
        || insertBelow(tab, 'control-panel', { ...card })) placed = true
  }
  if (!placed && next.tabs[0]?.cards) {
    const maxY = next.tabs[0].cards.reduce((m, c) => Math.max(m, c.y + c.h), 0)
    next.tabs[0].cards.push({ ...card, x: 0, y: maxY, w: 4 })
  }
  return next
}

const types = (tab) => tab.cards.map((c) => c.type)
const byType = (tab, t) => tab.cards.find((c) => c.type === t)

let n = 0
const check = (name, fn) => { fn(); n++; console.log(`  PASS  ${name}`) }

console.log('\n── v2 → v3 layout migration ─────────────────────────────────────')

// A typical saved v2 Control tab.
const v2 = {
  version: 2,
  tabs: [{
    id: 'control', name: 'Control',
    cards: [
      { key: 'control-panel#1', type: 'control-panel', x: 0, y: 0, w: 4, h: 8, props: {} },
      { key: 'gamepad#1', type: 'gamepad', x: 0, y: 8, w: 4, h: 11, props: {} },
      { key: 'imu#1', type: 'imu', x: 0, y: 19, w: 4, h: 4, props: {} },
      { key: 'joint-table#1', type: 'joint-table', x: 4, y: 0, w: 8, h: 9, props: {} },
    ],
  }],
}

const out = migrateV2(v2)

check('adds the Control method card', () =>
  assert.ok(types(out.tabs[0]).includes('control-method')))
check('adds the Quest card', () =>
  assert.ok(types(out.tabs[0]).includes('quest')))
check('keeps every original card', () => {
  for (const t of ['control-panel', 'gamepad', 'imu', 'joint-table']) {
    assert.ok(types(out.tabs[0]).includes(t), `lost ${t}`)
  }
})
check('stamps version 3', () => assert.strictEqual(out.version, 3))
check('does not mutate the input', () => assert.strictEqual(v2.version, 2))

check('Control method sits directly below the Control card', () => {
  const cp = byType(out.tabs[0], 'control-panel')
  const cm = byType(out.tabs[0], 'control-method')
  assert.strictEqual(cm.y, cp.y + cp.h)
  assert.strictEqual(cm.x, cp.x)
  assert.strictEqual(cm.w, cp.w)
})

check('cards below in the SAME column are pushed down, not overlapped', () => {
  const cm = byType(out.tabs[0], 'control-method')
  const gp = byType(out.tabs[0], 'gamepad')
  assert.ok(gp.y >= cm.y + cm.h, `gamepad y=${gp.y} overlaps method card ending at ${cm.y + cm.h}`)
})

check('a card in ANOTHER column keeps its position', () => {
  assert.strictEqual(byType(out.tabs[0], 'joint-table').y, 0)
  assert.strictEqual(byType(out.tabs[0], 'joint-table').x, 4)
})

check('no two cards in a column overlap after migration', () => {
  const col = out.tabs[0].cards.filter((c) => c.x === 0).sort((a, b) => a.y - b.y)
  for (let i = 1; i < col.length; i++) {
    assert.ok(col[i].y >= col[i - 1].y + col[i - 1].h,
      `${col[i].type} (y=${col[i].y}) overlaps ${col[i - 1].type}`)
  }
})

check('card keys are unique', () => {
  const keys = out.tabs[0].cards.map((c) => c.key)
  assert.strictEqual(new Set(keys).size, keys.length)
})

console.log('\n── idempotence and edge cases ───────────────────────────────────')

check('running it twice changes nothing', () =>
  assert.deepStrictEqual(migrateV2(out), out))

check('an already-v3 layout is returned untouched', () => {
  const v3 = { version: 3, tabs: [{ id: 'x', cards: [] }] }
  assert.strictEqual(migrateV2(v3), v3)
})

check('a layout with NO control-panel still gets the method card (never unreachable)', () => {
  const odd = { version: 2, tabs: [{ id: 'a', cards: [
    { key: 'imu#1', type: 'imu', x: 0, y: 0, w: 4, h: 4, props: {} }] }] }
  const r = migrateV2(odd)
  assert.ok(types(r.tabs[0]).includes('control-method'))
  assert.strictEqual(byType(r.tabs[0], 'control-method').y, 4)
})

check('a layout that already has the new cards does not duplicate them', () => {
  const already = { version: 2, tabs: [{ id: 'a', cards: [
    { key: 'control-panel#1', type: 'control-panel', x: 0, y: 0, w: 4, h: 6, props: {} },
    { key: 'control-method#1', type: 'control-method', x: 0, y: 6, w: 4, h: 8, props: {} },
  ] }] }
  const r = migrateV2(already)
  assert.strictEqual(r.tabs[0].cards.filter((c) => c.type === 'control-method').length, 1)
})

check('a tab with no cards array is skipped, not fatal', () => {
  const broken = { version: 2, tabs: [{ id: 'a' }, { id: 'b', cards: [
    { key: 'control-panel#1', type: 'control-panel', x: 0, y: 0, w: 4, h: 6, props: {} }] }] }
  const r = migrateV2(broken)
  assert.ok(types(r.tabs[1]).includes('control-method'))
})

check('a v2 layout with multiple tabs gets the card on the RIGHT tab', () => {
  const multi = { version: 2, tabs: [
    { id: 'robot', cards: [{ key: 'robot-view#1', type: 'robot-view', x: 0, y: 0, w: 12, h: 20, props: {} }] },
    { id: 'control', cards: [{ key: 'control-panel#1', type: 'control-panel', x: 0, y: 0, w: 4, h: 6, props: {} }] },
  ] }
  const r = migrateV2(multi)
  assert.ok(!types(r.tabs[0]).includes('control-method'), 'placed on the wrong tab')
  assert.ok(types(r.tabs[1]).includes('control-method'))
})


console.log('\n── v3 → v4: the Rest state card ─────────────────────────────────')

// The behaviour change this accompanies is invisible: the backend now damps at rest instead
// of going limp. An operator who wants the old behaviour has no way to ask for it unless the
// card actually lands in their saved layout, so "the card is present" is the real assertion.
const v3in = migrateV2(v2)
const v4 = migrateV3(v3in)
const tab4 = v4.tabs[0]

check('adds the Rest state card', () => assert.ok(types(tab4).includes('rest-mode')))
check('stamps version 4', () => assert.equal(v4.version, 4))
check('keeps every card v3 had', () =>
  assert.ok(types(v3in.tabs[0]).every((t) => types(tab4).includes(t))))
check('does not mutate the input', () =>
  assert.ok(!types(v3in.tabs[0]).includes('rest-mode')))
check('sits directly below Control method', () => {
  const m = byType(tab4, 'control-method'), r = byType(tab4, 'rest-mode')
  assert.equal(r.y, m.y + m.h)
  assert.equal(r.x, m.x)
})
check('pushes the column below it down rather than overlapping', () => {
  const r = byType(tab4, 'rest-mode'), g = byType(tab4, 'gamepad')
  assert.ok(g.y >= r.y + r.h, `gamepad y=${g.y} vs rest bottom ${r.y + r.h}`)
})
check('a card in another column keeps its position', () => {
  const j = byType(tab4, 'joint-table')
  assert.equal(j.x, 4); assert.equal(j.y, 0)
})
check('no two cards in a column overlap', () => {
  const col = tab4.cards.filter((c) => c.x === 0).sort((a, b) => a.y - b.y)
  for (let i = 1; i < col.length; i++) {
    assert.ok(col[i].y >= col[i - 1].y + col[i - 1].h,
      `${col[i - 1].type} and ${col[i].type} overlap`)
  }
})
check('running it twice changes nothing', () =>
  assert.deepEqual(migrateV3(migrateV3(v3in)), v4))
check('an already-v4 layout is returned untouched', () => {
  const already = { version: 4, tabs: [{ id: 'x', cards: [] }] }
  assert.equal(migrateV3(already), already)
})
check('falls back to Control when there is no Control method card', () => {
  const noMethod = { version: 3, tabs: [{ id: 'c', cards: [
    { key: 'control-panel#1', type: 'control-panel', x: 0, y: 0, w: 4, h: 8, props: {} },
  ] }] }
  const t = migrateV3(noMethod).tabs[0]
  assert.ok(types(t).includes('rest-mode'))
  assert.equal(byType(t, 'rest-mode').y, 8)
})
check('never leaves the card unreachable when neither anchor exists', () => {
  const orphan = { version: 3, tabs: [{ id: 'c', cards: [
    { key: 'imu#1', type: 'imu', x: 0, y: 0, w: 4, h: 4, props: {} },
  ] }] }
  assert.ok(types(migrateV3(orphan).tabs[0]).includes('rest-mode'))
})
check('does not duplicate a card that is already there', () => {
  const twice = migrateV3({ ...clone(v4), version: 3 })
  assert.equal(twice.tabs[0].cards.filter((c) => c.type === 'rest-mode').length, 1)
})
check('a tab with no cards array is skipped, not fatal', () => {
  const odd = { version: 3, tabs: [{ id: 'a' }, { id: 'b', cards: [
    { key: 'control-panel#1', type: 'control-panel', x: 0, y: 0, w: 4, h: 8, props: {} },
  ] }] }
  assert.ok(types(migrateV3(odd).tabs[1]).includes('rest-mode'))
})
check('card keys stay unique', () => {
  const keys = tab4.cards.map((c) => c.key)
  assert.equal(new Set(keys).size, keys.length)
})

console.log(`\n${n} passed, 0 failed`)
