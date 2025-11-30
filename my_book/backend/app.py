# import os
# import glob
# import time
# from typing import List
# from fastapi import FastAPI, HTTPException, BackgroundTasks
# from fastapi.middleware.cors import CORSMiddleware
# from pydantic import BaseModel
# import google.generativeai as genai
# from pinecone import Pinecone, ServerlessSpec
# from dotenv import load_dotenv
# import uvicorn

# # Load environment variables
# load_dotenv()

# app = FastAPI()

# # Allow CORS
# app.add_middleware(
#     CORSMiddleware,
#     allow_origins=["*"],
#     allow_credentials=True,
#     allow_methods=["*"],
#     allow_headers=["*"],
# )

# # --- Configuration ---
# GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
# PINECONE_API_KEY = os.getenv("PINECONE_API_KEY")

# if not GEMINI_API_KEY or not PINECONE_API_KEY:
#     print("WARNING: API keys not found in environment variables.")

# genai.configure(api_key=GEMINI_API_KEY)
# pc = Pinecone(api_key=PINECONE_API_KEY)
# INDEX_NAME = "book-assistant"

# # --- Agent Logic ---
# class BookAssistant:
#     def __init__(self):
#         self.model = genai.GenerativeModel('gemini-2.0-flash')
#         self.index = self._get_index()

#     def _get_index(self):
#         if INDEX_NAME not in pc.list_indexes().names():
#             # Create index if it doesn't exist (lazy creation)
#             try:
#                 pc.create_index(
#                     name=INDEX_NAME,
#                     dimension=768,
#                     metric="cosine",
#                     spec=ServerlessSpec(cloud="aws", region="us-east-1")
#                 )
#                 while not pc.describe_index(INDEX_NAME).status['ready']:
#                     time.sleep(1)
#             except Exception as e:
#                 print(f"Error creating index: {e}")
#         return pc.Index(INDEX_NAME)

#     def get_embedding(self, text: str, task_type="retrieval_query") -> List[float]:
#         result = genai.embed_content(
#             model="models/text-embedding-004",
#             content=text,
#             task_type=task_type
#         )
#         return result['embedding']

#     def query_vector_db(self, query: str, top_k: int = 3) -> List[str]:
#         try:
#             embedding = self.get_embedding(query)
#             results = self.index.query(vector=embedding, top_k=top_k, include_metadata=True)
#             return [match['metadata']['text'] for match in results['matches'] if 'text' in match['metadata']]
#         except Exception as e:
#             print(f"Vector DB Query Error: {e}")
#             return []

#     def generate_answer(self, query: str) -> str:
#         contexts = self.query_vector_db(query)
#         context_str = "\n\n".join(contexts)
        
#         prompt = f"""You are a helpful assistant for a book. Use the following context to answer the user's question.
# If the answer is not in the context, say you don't know.

# Context:
# {context_str}

# Question:
# {query}

# Answer:"""
        
#         try:
#             response = self.model.generate_content(prompt)
#             return response.text
#         except Exception as e:
#             return f"Error generating answer: {str(e)}"

#     def ingest_docs(self):
#         print("Starting ingestion...")
#         docs_path = "../docs/**/*.md"
#         files = glob.glob(docs_path, recursive=True)
        
#         vectors = []
#         for file_path in files:
#             try:
#                 with open(file_path, "r", encoding="utf-8") as f:
#                     content = f.read()
                
#                 chunks = content.split("\n\n")
#                 for i, chunk in enumerate(chunks):
#                     if len(chunk.strip()) < 50: continue
                    
#                     embedding = self.get_embedding(chunk, task_type="retrieval_document")
#                     vector_id = f"{os.path.basename(file_path)}_{i}"
#                     vectors.append({
#                         "id": vector_id,
#                         "values": embedding,
#                         "metadata": {"text": chunk, "source": file_path}
#                     })
                    
#                     if len(vectors) >= 50:
#                         self.index.upsert(vectors=vectors)
#                         vectors = []
#                         time.sleep(1)
#             except Exception as e:
#                 print(f"Error processing {file_path}: {e}")

#         if vectors:
#             self.index.upsert(vectors=vectors)
#         print("Ingestion complete.")

# # Initialize Agent
# book_assistant = BookAssistant()

# # --- API Endpoints ---
# class ChatRequest(BaseModel):
#     message: str

# class ChatResponse(BaseModel):
#     response: str

# @app.post("/chat", response_model=ChatResponse)
# async def chat_endpoint(request: ChatRequest):
#     response_text = book_assistant.generate_answer(request.message)
#     return ChatResponse(response=response_text)

# @app.post("/ingest")
# async def trigger_ingest(background_tasks: BackgroundTasks):
#     background_tasks.add_task(book_assistant.ingest_docs)
#     return {"status": "Ingestion started in background"}

# @app.get("/health")
# async def health_check():
#     return {"status": "ok"}

# if __name__ == "__main__":
#     uvicorn.run(app, host="0.0.0.0", port=8000)



# ---------------------------------------------------

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
        "You are a helpful assistant for a book. "
        "Always first call the rag_search tool to retrieve context. "
        "Then answer the question using only that context. "
        "If the answer is not in the context, say: 'I don't know from the book.'"
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
