
export interface PostPositionRequest {
  blocks: {
    color: string; // Lowercase string, e.g.,"red" | "green" | "blue"
    posX: number;  // Left-to-right grid coordinate; 0-indexed from leftmost
    posY: number;  // Top-to-bottom grid coordinate; 0-indexed from topmost
    posZ: number;  // Layer height coordinate; 0-indexed from lowest layer
  }[]
}

export interface GetPositionStatusResponse {
  status: "pending" | "running" | "succeeded" | "failed";
  message: string;
}

export interface GetMotionStatusResponse {
  status: "pending" | "running" | "succeeded" | "failed";
  message: string;
}

export async function postPosition(payload: PostPositionRequest) {
  return post<PostPositionRequest, void>(`/api/v1/position`, payload);
}

export async function getPositionStatus() {
  return get<GetPositionStatusResponse>(`/api/v1/position/status`);
}

export async function postMotionStart() {
  return post<void, void>(`/api/v1/motion/start`);
}

export async function postMotionStop() {
  return post<void, void>(`/api/v1/motion/stop`);
}

export async function getMotionStatus() {
  return get<GetMotionStatusResponse>(`/api/v1/position/status`);
}

async function post<TPayload, TResponse>(url: string, payload?: TPayload) {
  const body = payload ? JSON.stringify(payload) : undefined;
  const response = await fetch(url, { method: "POST", body });
  if (response.ok) return response.json() as Promise<TResponse>;
  throw new Error(`API request failed: ${response.status}, ${response.statusText}`);
}

async function get<TResponse>(url: string) {
  const response = await fetch(url, { method: "GET" });
  if (response.ok) return response.json() as Promise<TResponse>;
  throw new Error(`API request failed: ${response.status}, ${response.statusText}`);
}
