# Plan: MuJoCo Learning Lab Redesign

## Context

The user reviewed 3 design previews (Dark Terminal Lab, Notebook Explorer, Split-Screen Simulator) and wants a hybrid:
- **Look and feel** from Dark Terminal Lab (dark bg, teal accent, orange warm, monospace accents, glowing code blocks, control-room vibe)
- **Layout** from Split-Screen Simulator (lesson content on left, sticky interactive sandbox on right)
- **Navigation** via sidebar with lesson list (not a continuous page). Each lesson is its own view, switched via sidebar.
- **All content inline** - no links opening external pages. Theory, code, exercises, and sandbox all in one HTML file.

## Architecture: Single self-contained HTML file

**File:** `lessons/index.html` (replace existing)

### Layout

```
+-----------------+----------------------------+----------------------+
| SIDEBAR (220px) | LESSON CONTENT (scrollable)| SANDBOX (sticky,45%) |
| - Brand         | - Mental model theory      | - SVG visualization  |
| - Lesson list   | - Code examples            | - Parameter sliders  |
|   with dots     | - Prediction exercises     | - Readout values     |
| - Progress      | - Checkpoint               | - Step/Reset buttons |
+-----------------+----------------------------+----------------------+
```

- **Sidebar** (220px, sticky full-height): Brand + lesson nav with status dots + progress counter
- **Content pane** (~55%): scrollable, shows one lesson at a time based on sidebar selection
- **Sandbox pane** (~45%): sticky, each lesson has its own simulation that swaps when lesson changes
- **Mobile** (< 900px): sidebar → horizontal pill bar, split → vertical stack

### Design tokens (from Dark Terminal Lab preview)

- Dark bg (#0a0f0d), raised (#111916), surface (#151d19)
- Teal accent (#00e5c0) with glow effects
- Orange warm (#ff6b35) for exercises/warnings
- Monospace for numbers, labels, code, readouts
- Custom teal slider thumbs, code blocks with subtle glow
- Buttons: scale(0.96) on :active
- No em-dashes

### 3 lessons, all inline

**Lesson 01 - MJCF Hierarchy**
- Theory: XML → MjModel + MjData, bodies vs geoms, hierarchy
- Code: XML snippet + Python model inspection
- Sandbox: Pendulum SVG with height/rod-length/angle sliders, model counts readout
- Exercise: predict what changes when pos/size change

**Lesson 02 - Frames & Axes**
- Theory: local vs world coordinates, parent-child transformation
- Code: Python printing local vs world positions
- Sandbox: Two-body frame explorer with visible coordinate axes, local/world readout
- Exercise: predict world position from local values

**Lesson 03 - State & Stepping**
- Theory: mj_forward vs mj_step, state pipeline
- Code: Python stepping loop
- Sandbox: Pendulum with Step/Step x10/Reset, live time/qpos/qvel/qacc readout
- Exercise: predict time after N steps, predict motion direction

### Interactive features

- Sidebar lesson click → swap content and sandbox (no page reload)
- "Mark complete" button per lesson → updates sidebar dot + progress bar
- All simulations: vanilla JS + inline SVG, no external deps
- Lesson 03 uses semi-implicit Euler integration for stepping
- Sandbox readouts show labeled values with units

### Responsive

- Desktop (> 900px): sidebar + split content/sandbox
- Tablet (650-900px): horizontal pill bar top, content/sandbox stacked
- Mobile (< 650px): single column, sandbox follows content inline

### Accessibility

- Keyboard nav, focus-visible outlines (teal)
- ARIA labels on all interactive elements
- prefers-reduced-motion: disable transitions
- SVG title/desc elements

## File changes

1. **`lessons/index.html`** - complete rewrite (only file modified)

## Verification

1. Open in browser, verify sidebar switching works
2. All 3 sandbox simulations functional (sliders, buttons, SVG updates)
3. "Mark complete" updates sidebar dots + progress
4. Responsive at 1200px, 800px, 400px widths
5. Keyboard navigation through all controls
6. No external links - everything inline
