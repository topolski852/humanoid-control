/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,jsx}'],
  theme: {
    extend: {
      colors: {
        surface: {
          DEFAULT: '#0f1117',
          1: '#1a1d27',
          2: '#252836',
          3: '#2d3148',
        },
        accent: {
          DEFAULT: '#3b82f6',
          hover: '#2563eb',
          muted: 'rgba(59,130,246,0.15)',
        },
        online: '#22c55e',
        offline: '#4b5563',
        danger: '#ef4444',
        warn: '#f59e0b',
        // Keep these in step with COLORS in components/RobotWireframe.jsx — the legend text
        // is only useful if it is the same colour as the thing it names.
        twist: '#c084fc',
        claw: '#e8b04b',
      },
      fontFamily: {
        sans: ['Inter', 'system-ui', 'sans-serif'],
        mono: ['"JetBrains Mono"', '"Fira Code"', 'Consolas', 'monospace'],
      },
      animation: {
        'pulse-dot': 'pulse 2s cubic-bezier(0.4, 0, 0.6, 1) infinite',
      },
    },
  },
  plugins: [],
}
