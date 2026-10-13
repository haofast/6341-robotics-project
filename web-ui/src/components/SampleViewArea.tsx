import { Button } from "./ui/button";

export function SampleViewArea() {
  return (
    <div className="w-full flex flex-row items-center justify-center">
      <div className="flex flex-col gap-4">
        <div className="text-sm leading-loose">
          <h1>Project ready!</h1>
          <p>You may now add components and start building.</p>
          <p>We&apos;ve already added the button component for you.</p>
          <Button className="mt-2">Button</Button>
        </div>
        <div className="font-mono text-xs text-muted-foreground">
          (Press <kbd>d</kbd> to toggle dark mode)
        </div>
      </div>
    </div>
  )
}