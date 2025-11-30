# Book Assistant Feature Specification

## User Story
As a user, I want an assistant for this book that uses an agent, free embedding/model/004, and stores data in a vector database. When I ask about the book, the agent should provide answers. The assistant should have a logo in the bottom left, and when clicked, it should pop up a chat interface for interaction. The agent will be implemented using the OpenAI Agent's Python SDK, leveraging a Gemini free model. These changes should not alter any other existing web functionalities.

## Requirements

### Agent Core
- Utilize a large language model (LLM) for agent functionality.
  - **LLM**: Gemini free model.
- Integrate with OpenAI Agent's Python SDK: `https://openai.github.io/openai-agents-python/`.
- Employ a free embedding model/service: `embedding/model/004`.
- Store book-related data in a vector database to enable semantic search and retrieval for answering user queries.

### User Interface
- Display an assistant logo in the bottom left corner of the web interface.
- Upon clicking the logo, a chat interface should pop up, allowing users to interact with the assistant.

### Functionality
- The agent must be capable of answering user questions specifically about the book's content.

### Non-Functional Requirements
- The implementation of the book assistant must not introduce any changes or regressions to existing web functionalities or layout.
