/**
 * API Configuration
 * Centralized configuration for all API endpoints
 * Supports both development and production environments
 */

const getEnvironment = () => {
  if (typeof window === "undefined") {
    return "development";
  }

  const hostname = window.location.hostname || "";

  // Production environments (either of the deployed hosts or any vercel preview)
  if (
    hostname.includes("y-alpha-gold") ||
    hostname.includes("y-fopefocxn") ||
    hostname.includes("vercel.app")
  ) {
    return "production";
  }

  // Development
  return "development";
};

export const API_CONFIG = {
  development: {
    // Auth Server
    AUTH_SERVER:
      process.env.REACT_APP_AUTH_SERVER_URL || "http://localhost:4000",

    // Backend API
    BACKEND_BASE_URL:
      process.env.REACT_APP_BACKEND_URL || "http://localhost:8000",
    CHAT_ENDPOINT: "/chat",
    INGEST_ENDPOINT: "/ingest",
    HEALTH_ENDPOINT: "/health",
  },
  production: {
    // Auth Server - Set in environment variables or default to known hosts
    AUTH_SERVER:
      process.env.REACT_APP_AUTH_SERVER_URL || "https://y-alpha-gold.vercel.app",

    // Backend API - Set in environment variables; fallback resolved at runtime
    BACKEND_BASE_URL:
      process.env.REACT_APP_BACKEND_URL || "https://y-alpha-gold.vercel.app",
    CHAT_ENDPOINT: "/api/chat",
    INGEST_ENDPOINT: "/api/ingest",
    HEALTH_ENDPOINT: "/api/health",
  },
};

export const getApiConfig = () => {
  const env = getEnvironment();
  const config = API_CONFIG[env];

  // If running in browser and no explicit BACKEND_BASE_URL is provided for production,
  // prefer the current host (useful for Vercel previews / deployments)
  if (
    env === "production" &&
    typeof window !== "undefined" &&
    !process.env.REACT_APP_BACKEND_URL
  ) {
    const hostname = window.location.hostname || "";
    const inferred = `https://${hostname}`;
    return {
      ...config,
      BACKEND_BASE_URL: inferred,
      // If auth server not overridden, assume same host
      AUTH_SERVER: process.env.REACT_APP_AUTH_SERVER_URL || inferred,
    };
  }

  return config;
};

export const getFullUrl = (endpoint) => {
  const config = getApiConfig();
  return `${config.BACKEND_BASE_URL}${endpoint}`;
};

export const getChatUrl = () => {
  const config = getApiConfig();
  return `${config.BACKEND_BASE_URL}${config.CHAT_ENDPOINT}`;
};

export const getHealthUrl = () => {
  const config = getApiConfig();
  return `${config.BACKEND_BASE_URL}${config.HEALTH_ENDPOINT}`;
};

export const getIngestUrl = () => {
  const config = getApiConfig();
  return `${config.BACKEND_BASE_URL}${config.INGEST_ENDPOINT}`;
};
