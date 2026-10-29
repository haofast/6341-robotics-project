import type { LucideIcon } from "lucide-react"
import { Button } from "../ui/button"

interface ToolButtonProps {
  mainIcon: LucideIcon,
  accentIcon?: LucideIcon,
  iconColor?: string,
  disabled?: boolean
  badgeCount?: number
  action: () => void
}

export function ToolButton({
  mainIcon: MainIcon,
  accentIcon: AccentIcon,
  iconColor,
  disabled,
  badgeCount,
  action,
}: ToolButtonProps) {
  return (
    <Button
      className="relative p-5"
      variant="ghost"
      size="icon"
      onClick={action}
      disabled={disabled}
    >
      {AccentIcon && (
        <AccentIcon className="absolute top-0 right-0 text-muted-foreground" />
      )}
      <MainIcon className={`size-6 ${iconColor ?? "text-muted-foreground"}`} />
      {badgeCount !== undefined && (
        <div className="absolute bottom-1 left-1 text-muted-foreground font-bold">
          {badgeCount}
        </div>
      )}
    </Button>
  )
}
