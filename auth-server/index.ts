import { serve } from "@hono/node-server";
import { Hono } from "hono";
import { cors } from "hono/cors";
import { auth } from "./auth.js";
import { fileURLToPath } from "node:url";

const app = new Hono();

// Enable CORS for frontend (applies to all routes)
app.use(
  "*",
  cors({
    origin: (origin) => {
      if (!origin) return "http://localhost:3000";
      if (origin.endsWith(".vercel.app") || origin === "http://localhost:3000") {
        return origin;
      }
      return "http://localhost:3000";
    },
    allowHeaders: ["Content-Type", "Authorization"],
    allowMethods: ["POST", "GET", "OPTIONS"],
    exposeHeaders: ["Content-Length"],
    maxAge: 600,
    credentials: true,
  })
);

// Mount Better Auth handler
// Note: When deployed to Vercel at /api/index.ts, the /api prefix is stripped
// So we use /auth/** here, which becomes /api/auth/** when accessed
app.on(["POST", "GET"], "/auth/**", (c) => {
  return auth.handler(c.req.raw);
});

app.get("/", (c) => {
  return c.text("Auth Server is running!");
});

const port = 4000;
console.log(`Auth Server is running on http://localhost:${port}`);

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  serve({
    fetch: app.fetch,
    port,
  });
}

export default app;
