import { SidePanel } from "./components/SidePanel"
import { ToolPanel } from "./components/ToolPanel"
import { GridSpace } from "./components/workspace/GridSpace"
import { WorkspacePanel } from "./components/WorkspacePanel"

export function App() {
  return (
    <div className="w-screen h-screen">
      <GridSpace />
      <ToolPanel />
      <SidePanel />
      <WorkspacePanel />
    </div>
  )
}

export default App
