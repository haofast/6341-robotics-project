import { ControlPanel } from "./components/panels/ControlPanel"
import { ToolPanel } from "./components/panels/ToolPanel"
import { GridSpace } from "./components/grid/GridSpace"
import { WorkspaceInfoPanel } from "./components/panels/WorkspaceInfoPanel"

export function App() {
  return (
    <div className="w-screen h-screen">
      <GridSpace />
      <ToolPanel />
      <ControlPanel />
      <WorkspaceInfoPanel />
    </div>
  )
}

export default App
