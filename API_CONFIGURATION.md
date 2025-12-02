# API Configuration Guide

This document outlines all API endpoints and how to configure them for development and production.

## API Endpoints Overview

### 1. **Authentication Server**

- **Purpose**: Handles user authentication (sign up, sign in, sign out)
- **Development URL**: `http://localhost:4000`
- **Production URL**: Set via `REACT_APP_AUTH_SERVER_URL` environment variable
- **Endpoints**:
  - `POST /api/auth/sign-up` - User registration
  - `POST /api/auth/sign-in` - User login
  - `POST /api/auth/sign-out` - User logout
  - `GET /api/auth/session` - Get current session
- **Technology**: Hono + Better Auth
- **Configuration File**: `auth-server/auth.ts`, `auth-server/index.ts`
- **Client Configuration**: `my_book/src/lib/auth-client.js`

### 2. **Backend Chat API**

- **Purpose**: Handles chat messages with RAG (Retrieval-Augmented Generation)
- **Development URL**: `http://localhost:8000`
- **Production URL**: Set via `REACT_APP_BACKEND_URL` environment variable (typically `https://y-alpha-gold.vercel.app`)
- **Endpoints**:
  - `POST /chat` (dev) or `/api/chat` (prod) - Send chat message
  - `POST /ingest` (dev) or `/api/ingest` (prod) - Ingest documents
  - `GET /health` (dev) or `/api/health` (prod) - Health check
- **Technology**: FastAPI + OpenAI Agents SDK + Pinecone + Gemini
- **Configuration File**: `my_book/backend/app.py`
- **Client Configuration**: `my_book/src/lib/api-config.js`

## Configuration Files

### Environment Variables

#### Backend (my_book/.env.example)

```
REACT_APP_AUTH_SERVER_URL=http://localhost:4000
REACT_APP_BACKEND_URL=http://localhost:8000
```

#### Auth Server (auth-server/.env.example)

```
DATABASE_URL=postgresql://user:password@localhost:5432/book_auth
```

#### Backend Python (my_book/backend/.env)

```
GEMINI_API_KEY=your-gemini-api-key
PINECONE_API_KEY=your-pinecone-api-key
```

## Using the API Configuration

### In React Components

```javascript
import { getChatUrl, getHealthUrl, getIngestUrl } from "../lib/api-config";

// Get the full chat endpoint URL
const chatUrl = getChatUrl();
// Development: http://localhost:8000/chat
// Production: https://y-alpha-gold.vercel.app/api/chat

// Fetch example
const response = await fetch(getChatUrl(), {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({ message: "Hello" }),
});
```

## CORS Configuration

### Auth Server (auth-server/index.ts)

- Allows requests from:
  - `http://localhost:3000` (development)
  - `https://y-alpha-gold.vercel.app` (production)

### Auth Config (auth-server/auth.ts)

- Trusted origins include:
  - `http://localhost:3000`
  - `https://y-alpha-gold.vercel.app`

## Deployment Checklist

### Before Deploying to Production:

1. **Auth Server**

   - [ ] Deploy auth server to a Vercel or similar service
   - [ ] Note the deployed URL (e.g., `https://your-auth-server.vercel.app`)
   - [ ] Set up PostgreSQL database and `DATABASE_URL`

2. **Backend API**

   - [ ] Deploy backend to `https://y-alpha-gold.vercel.app` or your chosen URL
   - [ ] Set up environment variables: `GEMINI_API_KEY`, `PINECONE_API_KEY`
   - [ ] Update CORS settings in `auth-server/index.ts` and `auth.ts`

3. **Frontend**
   - [ ] Create `.env.production` with production URLs:
     ```
     REACT_APP_AUTH_SERVER_URL=https://your-auth-server.vercel.app
     REACT_APP_BACKEND_URL=https://y-alpha-gold.vercel.app
     ```
   - [ ] Deploy to Vercel

## Current Status

### Development Setup

- Auth Server: `http://localhost:4000`
- Backend: `http://localhost:8000`
- Frontend: `http://localhost:3000`

### Production URLs

- Frontend: `https://y-alpha-gold.vercel.app`
- Backend: `https://y-alpha-gold.vercel.app` (or separate domain)
- Auth Server: ⚠️ **Needs to be set** - Deploy auth server separately

## Testing the Connection

### Test Auth Server

```bash
curl -X GET http://localhost:4000/
# Expected: Auth Server is running!
```

### Test Backend Health

```bash
curl -X GET http://localhost:8000/health
# Expected: {"status": "ok"}
```

### Test Chat Endpoint

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello"}'
```

## API Configuration Strategy

The `api-config.js` file automatically detects the environment based on the hostname:

- If `hostname === 'y-alpha-gold.vercel.app'` → Uses production URLs
- Otherwise → Uses development URLs from environment variables

This allows for seamless switching between development and production without code changes.
