import { ControlsPanel } from "./components/panels/ControlsPanel"
import { WorkspaceToolsPanel } from "./components/panels/WorkspaceToolsPanel"
import { GridSpace } from "./components/workspace/GridSpace"
import { WorkspaceInfoPanel } from "./components/panels/WorkspaceInfoPanel"

export function App() {
  return (
    <div className="w-screen h-screen">
      <GridSpace />
      <WorkspaceToolsPanel />
      <WorkspaceInfoPanel />
      <ControlsPanel />
    </div>
  )
}

export default App
