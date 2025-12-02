
import os
import glob
import time
from typing import List

from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv

import uvicorn
from pinecone import Pinecone, ServerlessSpec

# --- OpenAI Agents SDK ---
from agents import Agent, Runner, function_tool
from agents import (
    AsyncOpenAI,
    OpenAIChatCompletionsModel,
    RunConfig
)
from agents import set_default_openai_client
from agents import set_tracing_disabled

set_tracing_disabled(True)
# -----------------------------
# Load environment variables
# -----------------------------
load_dotenv()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
PINECONE_API_KEY = os.getenv("PINECONE_API_KEY")

if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY is not set in .env")

if not PINECONE_API_KEY:
    raise ValueError("PINECONE_API_KEY is not set in .env")

# -----------------------------
# Gemini Client (OpenAI Compatible)
# -----------------------------
external_client = AsyncOpenAI(
    api_key=GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

# Make Gemini the default client for Agents SDK
set_default_openai_client(external_client)

gemini_model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash",
    openai_client=external_client
)

# -----------------------------
# Pinecone Setup
# -----------------------------
pc = Pinecone(api_key=PINECONE_API_KEY)
INDEX_NAME = "book-assistant"

if INDEX_NAME not in pc.list_indexes().names():
    pc.create_index(
        name=INDEX_NAME,
        dimension=768,
        metric="cosine",
        spec=ServerlessSpec(cloud="aws", region="us-east-1")
    )
    while not pc.describe_index(INDEX_NAME).status["ready"]:
        time.sleep(1)

index = pc.Index(INDEX_NAME)

# -----------------------------
# RAG Search Tool (Agent Tool)
# -----------------------------
@function_tool
async def rag_search(query: str, top_k: int = 3) -> List[str]:
    """
    Search Pinecone using Gemini embeddings.
    """

    # Get Gemini embeddings
    emb = await external_client.embeddings.create(
        model="models/text-embedding-004",
        input=query
    )
    vector = emb.data[0].embedding

    # Pinecone search
    results = index.query(
        vector=vector,
        top_k=top_k,
        include_metadata=True
    )

    return [
        match["metadata"].get("text", "")
        for match in results.matches
        if "text" in match["metadata"]
    ]

# -----------------------------
# Agent Setup
# -----------------------------
agent = Agent(
    name="BookAssistant",
    instructions=(
        "You are a specialized Book Assistant designed to help users understand the content of this book.\n"
        "Your primary goal is to provide accurate information based ONLY on the book's content.\n\n"
        
        "CORE INSTRUCTIONS:\n"
        "1. ALWAYS start by using the 'rag_search' tool. For broad questions like 'tell me about the book' or 'chapter 1', use a descriptive query.\n"
        "2. Answer the question using ONLY the information returned by the 'rag_search' tool.\n"
        "3. If the answer is NOT found in the retrieved context, say: 'I am specialized to answer questions about this book only. I cannot find that information in the book's content.'\n"
        "4. Be helpful, concise, and professional.\n\n"
        
        "SPECIFIC SCENARIOS:\n"
        "- Greeting: If the user says hello/hi, greet them warmly and ask how you can help with the book.\n"
        "- Book Overview: If asked about the book's purpose, summarize what the book teaches based on the context.\n"
        "- Chapter Requests: If the user asks for a specific chapter (e.g., 'Chapter 1'), you MUST search for 'Chapter 1 content' or 'Chapter 1 summary' to retrieve relevant sections. Provide a comprehensive summary of the topics covered in that chapter.\n"
        "- Off-topic: Politely redirect to the book's topics."
    ),
    model=gemini_model,
    tools=[rag_search],
)

# -----------------------------
# Document Ingestion
# -----------------------------
async def ingest_docs():
    print("Starting ingestion...")
    docs_path = "../docs/**/*.md"
    files = glob.glob(docs_path, recursive=True)
    vectors = []

    for file_path in files:
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()

            chunks = content.split("\n\n")

            for i, chunk in enumerate(chunks):
                if len(chunk.strip()) < 50:
                    continue

                # Gemini embeddings
                emb = await external_client.embeddings.create(
                    model="google/text-embedding-004",
                    input=chunk
                )
                vec = emb.data[0].embedding

                vectors.append({
                    "id": f"{os.path.basename(file_path)}_{i}",
                    "values": vec,
                    "metadata": {"text": chunk, "source": file_path}
                })

                if len(vectors) >= 50:
                    index.upsert(vectors=vectors)
                    vectors = []
                    time.sleep(1)

        except Exception as e:
            print(f"Error processing {file_path}: {e}")

    if vectors:
        index.upsert(vectors=vectors)

    print("Ingestion complete.")

# -----------------------------
# FastAPI Setup
# -----------------------------
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    message: str

class ChatResponse(BaseModel):
    response: str

# -----------------------------
# Chat Endpoint
# -----------------------------
@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    result = await Runner.run(agent, request.message)
    return ChatResponse(response=result.final_output)

# -----------------------------
# Ingest Endpoint
# -----------------------------
@app.post("/ingest")
async def trigger_ingest(background_tasks: BackgroundTasks):
    background_tasks.add_task(ingest_docs)
    return {"status": "Ingestion started in background"}

# -----------------------------
# Health Check
# -----------------------------
@app.get("/health")
async def health_check():
    return {"status": "ok"}

# -----------------------------
# Run Application
# -----------------------------
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
