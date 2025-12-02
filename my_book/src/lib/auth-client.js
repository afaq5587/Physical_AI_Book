import { createAuthClient } from "better-auth/react";

// Resolve auth server URL with this priority:
// 1. REACT_APP_AUTH_SERVER_URL (recommended for production)
// 2. If running in browser and the hostname looks like the deployed site, use that
// 3. Fallback to localhost for development
const getAuthServerURL = () => {
  // Use the specific auth server URL provided by the user
  // Note: When deployed to Vercel, the auth routes are at /api/auth/**
  const productionURL = "https://y-alpha-gold.vercel.app/api";

  if (typeof window !== "undefined" && window.location.hostname !== "localhost") {
    return productionURL;
  }

  return "http://localhost:4000";
};

export const authClient = createAuthClient({
  baseURL: getAuthServerURL(),
});
