import { useCallback, useEffect, useState } from 'react'
import { DEFAULT_LAYOUT, GRID_COLS, widgetById } from './widgets'

// Dashboard layout: tabs, the cards on them, and each card's position and settings.
//
// All storage is behind this hook. Today that is localStorage — per browser, instant, no server
// round-trip. If layouts should later follow the ROBOT rather than the browser, only `read` and
// `write` change.
//
// The layout OWNS membership; the widget catalog is only a menu. Cards are instances keyed by
// `type#N`, so the same widget can appear twice on a tab with different settings.

const KEY = 'humanoid_dashboard_v2'
const LEGACY_KEY = 'humanoid_dashboard_layouts_v1'

const clone = (o) => JSON.parse(JSON.stringify(o))

function read() {
  try {
    const raw = localStorage.getItem(KEY)
    if (raw) {
      const parsed = JSON.parse(raw)
      if (parsed?.tabs?.length) return migrateV2(parsed)
    }
    const legacy = localStorage.getItem(LEGACY_KEY)
    if (legacy) return migrateV1(JSON.parse(legacy))
  } catch {
    // Private window, cleared storage, or a corrupt value. Falling back to the shipped layout
    // is always safe — nothing here is data the operator cannot rebuild.
  }
  return clone(DEFAULT_LAYOUT)
}

function write(layout) {
  try {
    localStorage.setItem(KEY, JSON.stringify(layout))
    return true
  } catch {
    return false          // quota or blocked storage: the session still works, it just won't persist
  }
}

/** v1 was {tabId: [{i,x,y,w,h}]} against a hardcoded tab/widget set. Carry positions across
 *  rather than resetting someone's arrangement on upgrade. */
function migrateV1(old) {
  const next = clone(DEFAULT_LAYOUT)
  for (const tab of next.tabs) {
    const saved = old?.[tab.id]
    if (!Array.isArray(saved)) continue
    const byType = new Map(saved.map((l) => [l.i, l]))
    for (const card of tab.cards) {
      const l = byType.get(card.type)          // v1 keyed by widget id, not instance
      if (l && Number.isFinite(l.x)) Object.assign(card, { x: l.x, y: l.y, w: l.w, h: l.h })
    }
  }
  return next
}

/** v2 → v3: the Control card lost its motion section to the new Control method card.
 *
 *  Until now `version` was decorative — it was written and never read, so a saved layout came
 *  back verbatim however far the catalog had moved on. That was harmless while cards only ever
 *  gained features; it stops being harmless the moment a card LOSES one. Anyone who had ever
 *  touched their dashboard would get the motion-less Control card with no Control method card
 *  anywhere, and therefore no route to Run policy at all, with nothing on screen explaining why.
 *
 *  INSERT ONLY. Every existing card, position and setting is preserved: `control-method` goes
 *  directly below the Control card and `quest` below the Xbox card, with everything lower on
 *  that column pushed down. If a layout has no Control card at all, the method card is appended
 *  to the first tab rather than dropped — the controls must never be unreachable.
 */
function migrateV2(layout) {
  if ((layout.version ?? 2) >= 3) return layout
  const next = clone(layout)
  next.version = 3

  const insertBelow = (tab, anchorType, card) => {
    const anchor = tab.cards.find((c) => c.type === anchorType)
    if (!anchor) return false
    if (tab.cards.some((c) => c.type === card.type)) return true   // already present
    const y = anchor.y + anchor.h
    // Push down anything below the anchor IN THE SAME COLUMN. Cards in other columns keep
    // their position — shifting the whole tab would rearrange a layout the user chose.
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
                      title: 'Control method', h: 8, props: {} })) {
      placedMethod = true
    }
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

// --- helpers ---------------------------------------------------------------

const allCards = (layout) =>
  layout.tabs.flatMap((t) => t.cards.flatMap((c) => [c, ...(c.children || [])]))

/** Next free instance key for a type, across the WHOLE layout so keys are never reused. */
function mintKey(layout, type) {
  let max = 0
  for (const c of allCards(layout)) {
    const m = String(c.key).match(new RegExp(`^${type}#(\\d+)$`))
    if (m) max = Math.max(max, Number(m[1]))
  }
  return `${type}#${max + 1}`
}

/** First row below everything already placed, so a new card never lands on top of one. */
function nextFreeY(cards) {
  return cards.reduce((m, c) => Math.max(m, (c.y || 0) + (c.h || 1)), 0)
}

function newCard(layout, type, cards) {
  const w = widgetById(type)
  const d = w?.defaultLayout || { w: 4, h: 6 }
  const card = {
    key: mintKey(layout, type),
    type,
    title: w?.title || type,
    x: 0, y: nextFreeY(cards), w: Math.min(d.w, GRID_COLS), h: d.h,
    props: {},
  }
  if (w?.container) card.children = []
  return card
}

// --- hook ------------------------------------------------------------------

export function useLayouts() {
  const [layout, setLayout] = useState(read)

  // Another tab of the same browser editing its dashboard should not silently diverge.
  useEffect(() => {
    const onStorage = (e) => { if (e.key === KEY) setLayout(read()) }
    window.addEventListener('storage', onStorage)
    return () => window.removeEventListener('storage', onStorage)
  }, [])

  // Every mutation goes through here: apply to a copy, persist, publish. Keeping it in one
  // place means no operation can forget to save.
  const mutate = useCallback((fn) => {
    setLayout((prev) => {
      const next = clone(prev)
      const result = fn(next)
      if (result === false) return prev        // operation refused; leave state untouched
      write(next)
      return next
    })
  }, [])

  const findTab = (l, tabId) => l.tabs.find((t) => t.id === tabId)

  // --- grid ---------------------------------------------------------------
  const saveGrid = useCallback((tabId, grid) => mutate((l) => {
    const tab = findTab(l, tabId)
    if (!tab) return false
    const byKey = new Map(grid.map((g) => [g.i, g]))
    for (const card of tab.cards) {
      const g = byKey.get(card.key)
      if (g) Object.assign(card, { x: g.x, y: g.y, w: g.w, h: g.h })
    }
  }), [mutate])

  // --- cards --------------------------------------------------------------
  const addCard = useCallback((tabId, type) => mutate((l) => {
    const tab = findTab(l, tabId)
    if (!tab) return false
    tab.cards.push(newCard(l, type, tab.cards))
  }), [mutate])

  const removeCard = useCallback((tabId, key) => mutate((l) => {
    const tab = findTab(l, tabId)
    if (!tab) return false
    tab.cards = tab.cards.filter((c) => c.key !== key)
  }), [mutate])

  const duplicateCard = useCallback((tabId, key) => mutate((l) => {
    const tab = findTab(l, tabId)
    const src = tab?.cards.find((c) => c.key === key)
    if (!src) return false
    const copy = clone(src)
    copy.key = mintKey(l, src.type)
    copy.y = nextFreeY(tab.cards)
    // Children are instances too and must not share keys with the original's.
    copy.children = (copy.children || []).map((ch) => ({ ...ch, key: mintKey(l, ch.type) }))
    tab.cards.push(copy)
  }), [mutate])

  /** Patch a card's title/props. Works on nested children too. */
  const updateCard = useCallback((tabId, key, patch) => mutate((l) => {
    const tab = findTab(l, tabId)
    if (!tab) return false
    for (const c of tab.cards) {
      if (c.key === key) { Object.assign(c, patch); return }
      const child = (c.children || []).find((x) => x.key === key)
      if (child) { Object.assign(child, patch); return }
    }
    return false
  }), [mutate])

  // --- group children -----------------------------------------------------
  const addChild = useCallback((tabId, groupKey, type) => mutate((l) => {
    const tab = findTab(l, tabId)
    const group = tab?.cards.find((c) => c.key === groupKey)
    if (!group) return false
    group.children = group.children || []
    const w = widgetById(type)
    group.children.push({ key: mintKey(l, type), type, title: w?.title || type, props: {} })
  }), [mutate])

  /** Move a child out onto the grid, or drop it entirely. */
  const popOutChild = useCallback((tabId, groupKey, childKey, discard = false) => mutate((l) => {
    const tab = findTab(l, tabId)
    const group = tab?.cards.find((c) => c.key === groupKey)
    const child = group?.children?.find((c) => c.key === childKey)
    if (!child) return false
    group.children = group.children.filter((c) => c.key !== childKey)
    if (discard) return
    const w = widgetById(child.type)
    const d = w?.defaultLayout || { w: 3, h: 5 }
    // Keeps its key and settings — popping out should feel like moving the card, not
    // replacing it with a fresh one.
    tab.cards.push({ ...child, x: 0, y: nextFreeY(tab.cards), w: d.w, h: d.h })
  }), [mutate])

  // --- tabs ---------------------------------------------------------------
  const addTab = useCallback((name = 'New tab') => mutate((l) => {
    const id = `tab-${Date.now().toString(36)}`
    l.tabs.push({ id, name, cards: [] })
  }), [mutate])

  const renameTab = useCallback((tabId, name) => mutate((l) => {
    const tab = findTab(l, tabId)
    if (!tab) return false
    tab.name = name || tab.name
  }), [mutate])

  const removeTab = useCallback((tabId) => mutate((l) => {
    // A dashboard with no tabs has no way back to one, so the last tab stays.
    if (l.tabs.length <= 1) return false
    l.tabs = l.tabs.filter((t) => t.id !== tabId)
  }), [mutate])

  const duplicateTab = useCallback((tabId) => mutate((l) => {
    const i = l.tabs.findIndex((t) => t.id === tabId)
    if (i < 0) return false
    const copy = clone(l.tabs[i])
    copy.id = `tab-${Date.now().toString(36)}`
    copy.name = `${copy.name} copy`
    delete copy.badge
    for (const c of copy.cards) {
      c.key = mintKey(l, c.type)
      c.children = (c.children || []).map((ch) => ({ ...ch, key: mintKey(l, ch.type) }))
    }
    l.tabs.splice(i + 1, 0, copy)
  }), [mutate])

  const moveTab = useCallback((tabId, delta) => mutate((l) => {
    const i = l.tabs.findIndex((t) => t.id === tabId)
    const j = i + delta
    if (i < 0 || j < 0 || j >= l.tabs.length) return false
    const [t] = l.tabs.splice(i, 1)
    l.tabs.splice(j, 0, t)
  }), [mutate])

  const restoreDefaults = useCallback(() => {
    const fresh = clone(DEFAULT_LAYOUT)
    write(fresh)
    try { localStorage.removeItem(LEGACY_KEY) } catch { /* ignore */ }
    setLayout(fresh)
  }, [])

  return {
    layout,
    saveGrid,
    addCard, removeCard, duplicateCard, updateCard,
    addChild, popOutChild,
    addTab, renameTab, removeTab, duplicateTab, moveTab,
    restoreDefaults,
  }
}
