import { Console } from "./Console";
import { Button } from "./ui/button";
import { Card } from "./ui/card";

export function SidePanel() {

  return (
    <Card className="fixed top-6 right-6 bottom-6 w-80 p-4 overflow-y-auto bg-background/75 backdrop-blur">
      <div className="space-y-2">
        <div className="text-[10px] font-light tracking-widest">
          POSITION CONSOLE
        </div>
        <Console logs={[
          {
            type: "info",
            timestamp: new Date(),
            text: "end effector at (0,0,0)",
          }
        ]} />
      </div>
      <hr />
      <div className="space-y-2">
        <div className="text-[10px] font-light tracking-widest">
          LOG CONSOLE
        </div>
        <Console logs={[
          {
            type: "info",
            timestamp: new Date(),
            text: "positioning uploaded",
          }
        ]} />
      </div>
      <hr />
      <div className="space-y-2">
        <div className="text-[10px] font-light tracking-widest">
          CONTROLS
        </div>
        <div className="flex flex-row gap-2">
          <Button className="flex-grow" variant="default">
            Start
          </Button>
          <Button className="flex-grow" variant="outline">
            Stop
          </Button>
        </div>
      </div>
    </Card>
  )
}