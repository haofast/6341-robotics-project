import { Console } from "./Console";
import { Button } from "./ui/button";
import { Card } from "./ui/card";

export function SidePanel() {
  return (
    <Card className="w-100 p-4">
      <div className="space-y-2">
        <div className="text-[10px] font-light tracking-widest">
          ITEMS
        </div>
        <ul>
          <li>12 Red Blocks</li>
          <li>6 Blue Blocks</li>
          <li>4 Green Blocks</li>
        </ul>
        <Button className="w-full" variant="outline">
          More Info
        </Button>
      </div>
      <hr />
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