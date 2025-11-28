import os
import glob
import google.generativeai as genai
from pinecone import Pinecone, ServerlessSpec
from dotenv import load_dotenv
import time

load_dotenv()

# Configure Gemini
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

# Configure Pinecone
pc = Pinecone(api_key=os.getenv("PINECONE_API_KEY"))
index_name = "book-assistant"

# Create index if not exists
if index_name not in pc.list_indexes().names():
    pc.create_index(
        name=index_name,
        dimension=768, # Dimension for text-embedding-004
        metric="cosine",
        spec=ServerlessSpec(
            cloud="aws",
            region="us-east-1"
        )
    )
    while not pc.describe_index(index_name).status['ready']:
        time.sleep(1)

index = pc.Index(index_name)

def get_embedding(text):
    result = genai.embed_content(
        model="models/text-embedding-004",
        content=text,
        task_type="retrieval_document"
    )
    return result['embedding']

def ingest_docs():
    docs_path = "../docs/**/*.md"
    files = glob.glob(docs_path, recursive=True)
    
    print(f"Found {len(files)} markdown files.")
    
    vectors = []
    
    for file_path in files:
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
            
        # Simple chunking by paragraphs for now
        chunks = content.split("\n\n")
        
        for i, chunk in enumerate(chunks):
            if len(chunk.strip()) < 50: # Skip small chunks
                continue
                
            try:
                embedding = get_embedding(chunk)
                vector_id = f"{os.path.basename(file_path)}_{i}"
                vectors.append({
                    "id": vector_id,
                    "values": embedding,
                    "metadata": {"text": chunk, "source": file_path}
                })
                
                # Batch upsert to avoid hitting limits
                if len(vectors) >= 50:
                    index.upsert(vectors=vectors)
                    vectors = []
                    time.sleep(1) # Rate limiting precaution
                    
            except Exception as e:
                print(f"Error processing chunk in {file_path}: {e}")

    if vectors:
        index.upsert(vectors=vectors)
        
    print("Ingestion complete.")

if __name__ == "__main__":
    ingest_docs()
