import ControlPanel from '../components/ControlPanel'
import GamepadPanel from '../components/GamepadPanel'
import ImuPanel from '../components/ImuPanel'
import JointTable from '../components/JointTable'
import RobotMini from '../components/RobotMini'
import RobotView from '../components/RobotView'
import CalibrationPanel from '../components/CalibrationPanel'
import ManualPanel from '../components/ManualPanel'
import SettingsPanel from '../components/SettingsPanel'
import JointCard from '../components/JointCard'

// The widget CATALOG — every card that can be placed, and nothing about where any of them is.
//
// Membership lives in the saved layout (see useLayouts.js): a tab holds card INSTANCES, each
// naming a `type` looked up here. That inversion is what makes cards addable, removable and
// duplicable — the catalog is a menu, not an arrangement.
//
// `id` is a permanent identifier: renaming one orphans every saved layout that mentions it.
//
// `settings` is a small schema the settings popover renders generically, so adding an option to
// a widget never means touching the dashboard. `size` is injected automatically for any widget
// declaring `tiers`.
//
// `tiers` gives up to three renderings by height — below `mini` rows a glanceable indicator,
// below `compact` the essentials, otherwise everything. Only consulted when the card's `size`
// setting is 'auto'.

export const GRID_COLS = 12
export const ROW_HEIGHT = 30

const SIZE_SETTING = {
  key: 'size',
  label: 'Detail',
  type: 'select',
  options: [
    { value: 'auto', label: 'Auto (by size)' },
    { value: 'full', label: 'Full' },
    { value: 'compact', label: 'Compact' },
    { value: 'mini', label: 'Mini' },
  ],
  default: 'auto',
}

export const WIDGETS = [
  {
    id: 'control-panel',
    title: 'Control',
    group: 'Control',
    description: 'Connect, arm and run motion. Mini shows state only — for gamepad driving.',
    defaultLayout: { w: 4, h: 8 },
    minW: 2, minH: 2,
    tiers: { mini: 5, compact: 8 },
    render: ({ deadmanConnected, size }) =>
      <ControlPanel deadmanConnected={deadmanConnected} size={size} />,
  },
  {
    id: 'manual-panel',
    title: 'Manual control',
    group: 'Control',
    description: 'Saved poses, go-to-pose and capture-and-hold.',
    defaultLayout: { w: 8, h: 11 },
    minW: 3, minH: 4,
    render: ({ deadmanConnected }) => <ManualPanel deadmanConnected={deadmanConnected} />,
  },
  {
    id: 'calibration-panel',
    title: 'Calibration',
    group: 'Control',
    description: 'Per-joint zeroing and the operator override.',
    defaultLayout: { w: 8, h: 12 },
    minW: 3, minH: 4,
    render: () => <CalibrationPanel />,
  },
  {
    id: 'gamepad',
    title: 'Controller',
    group: 'Diagnostics',
    description: 'Live button and axis state, plus the active control mode.',
    defaultLayout: { w: 4, h: 11 },
    minW: 2, minH: 2,
    tiers: { mini: 6, compact: 11 },
    render: () => <GamepadPanel />,
  },
  {
    id: 'imu',
    title: 'IMU',
    group: 'Diagnostics',
    description: 'Base attitude from gravity.',
    defaultLayout: { w: 4, h: 4 },
    minW: 2, minH: 2,
    render: () => <ImuPanel />,
  },
  {
    id: 'joint-table',
    title: 'Joints',
    group: 'Robot',
    description: 'Every configured joint in one table.',
    defaultLayout: { w: 8, h: 9 },
    minW: 2, minH: 2,
    tiers: { mini: 5, compact: 9 },
    render: ({ size }) => <JointTable size={size} />,
  },
  {
    id: 'joint',
    title: 'Single joint',
    group: 'Robot',
    description: 'One joint as a large read-out. Inside a group it becomes a table row.',
    defaultLayout: { w: 2, h: 5 },
    minW: 1, minH: 3,
    childOf: 'group',              // may also live inside a group
    bare: true,                    // JointCard renders its own card chrome
    settings: [
      // Options are filled at render time from live telemetry, so the list is whatever the
      // configured layout actually has rather than a hardcoded joint set.
      { key: 'joint', label: 'Joint', type: 'select', optionsFrom: 'joints' },
    ],
    render: ({ props }) => <JointCard props={props} />,
  },
  {
    id: 'group',
    title: 'Group',
    group: 'Layout',
    description: 'Holds other cards and stacks them as rows. Add cards with its + button.',
    defaultLayout: { w: 6, h: 8 },
    minW: 2, minH: 3,
    container: true,
    bare: true,                    // GroupCard renders its own card chrome
    settings: [
      {
        key: 'columns', label: 'Row detail', type: 'select', default: 'full',
        options: [
          { value: 'full', label: 'All columns' },
          { value: 'compact', label: 'State, position, velocity' },
          { value: 'minimal', label: 'Name and position' },
        ],
      },
    ],
    // Rendered by Dashboard directly (it needs the layout mutators), not here.
    render: () => null,
  },
  {
    id: 'robot-mini',
    title: 'Robot pose',
    group: 'Robot',
    description: 'Compact wireframe from live encoders.',
    defaultLayout: { w: 4, h: 11 },
    minW: 2, minH: 2,
    tiers: { mini: 5, compact: 9 },
    render: () => <RobotMini />,
  },
  {
    id: 'robot-view',
    title: 'Robot (full)',
    group: 'Robot',
    description: 'Wireframe plus the frame-critical joint readout.',
    defaultLayout: { w: 12, h: 20 },
    minW: 4, minH: 8,
    bare: true,
    render: () => <RobotView />,
  },
  {
    id: 'settings-panel',
    title: 'Robot settings',
    group: 'Control',
    description: 'Which limbs are attached, and the IMU flag.',
    defaultLayout: { w: 7, h: 15 },
    minW: 3, minH: 5,
    bare: true,
    render: () => <SettingsPanel />,
  },
]

export const widgetById = (id) => WIDGETS.find((w) => w.id === id)

/** Settings schema for a widget, with the common `size` option folded in where it applies. */
export function settingsFor(widget) {
  if (!widget) return []
  return [...(widget.settings || []), ...(widget.tiers ? [SIZE_SETTING] : [])]
}

// The shipped dashboard. Seeds first run and backs "Restore default layout". Kept identical to
// the pre-configurable arrangement so an existing user sees no change until they edit something.
export const DEFAULT_LAYOUT = {
  version: 2,
  tabs: [
    {
      id: 'control', name: 'Control',
      cards: [
        { key: 'control-panel#1', type: 'control-panel', title: 'Control', x: 0, y: 0, w: 4, h: 8, props: {} },
        { key: 'gamepad#1', type: 'gamepad', title: 'Controller', x: 0, y: 8, w: 4, h: 11, props: {} },
        { key: 'imu#1', type: 'imu', title: 'IMU', x: 0, y: 19, w: 4, h: 4, props: {} },
        { key: 'robot-mini#1', type: 'robot-mini', title: 'Robot pose', x: 0, y: 23, w: 4, h: 11, props: {} },
        { key: 'joint-table#1', type: 'joint-table', title: 'Joints', x: 4, y: 0, w: 8, h: 9, props: {} },
      ],
    },
    {
      id: 'robot', name: 'Robot',
      cards: [
        { key: 'robot-view#1', type: 'robot-view', title: 'Robot', x: 0, y: 0, w: 12, h: 20, props: {} },
      ],
    },
    {
      id: 'manual', name: 'Manual',
      cards: [
        { key: 'robot-mini#2', type: 'robot-mini', title: 'Robot pose', x: 0, y: 0, w: 4, h: 11, props: {} },
        { key: 'manual-panel#1', type: 'manual-panel', title: 'Manual control', x: 4, y: 0, w: 8, h: 11, props: {} },
      ],
    },
    {
      id: 'calibration', name: 'Calibration', badge: 'uncalibrated',
      cards: [
        { key: 'robot-mini#3', type: 'robot-mini', title: 'Robot pose', x: 0, y: 0, w: 4, h: 11, props: {} },
        { key: 'calibration-panel#1', type: 'calibration-panel', title: 'Calibration', x: 4, y: 0, w: 8, h: 12, props: {} },
      ],
    },
    {
      id: 'settings', name: 'Settings',
      cards: [
        { key: 'settings-panel#1', type: 'settings-panel', title: 'Robot settings', x: 0, y: 0, w: 7, h: 15, props: {} },
      ],
    },
  ],
}
