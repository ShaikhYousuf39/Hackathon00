"""
FastAPI backend server for the RAG Chatbot.
Provides /chat and /feedback endpoints with database persistence.
"""

import os
import sys
import time
import logging
from datetime import datetime
from typing import Optional
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

# Add parent directory to path to import root-level modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from agent import RAGAgent
from retriever import RAGRetriever
from config import config

from .schemas import (
    ChatRequest,
    ChatResponse,
    FeedbackRequest,
    FeedbackResponse,
    HealthResponse,
    ErrorResponse
)

from ..database.database import db_manager, init_database, get_db
from ..database.models import (
    ChatSession,
    UserQuery,
    ChatbotResponse,
    UserFeedback
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Global RAG agent instance
rag_agent: Optional[RAGAgent] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan event handler for startup and shutdown."""
    # Startup
    logger.info("Starting up FastAPI application...")

    # Initialize database
    try:
        init_database()
        logger.info("Database initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize database: {e}")
        raise

    # Initialize RAG Agent
    global rag_agent
    try:
        rag_agent = RAGAgent()
        logger.info("RAG Agent initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize RAG Agent: {e}")
        raise

    yield

    # Shutdown
    logger.info("Shutting down FastAPI application...")
    if rag_agent:
        rag_agent.close()
        logger.info("RAG Agent closed")


# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG Chatbot API",
    description="Backend API for RAG-powered chatbot with Qdrant retrieval and Gemini responses",
    version="1.0.0",
    lifespan=lifespan
)

# Add rate limiter
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Configure CORS
origins = [
    "http://localhost:3000",  # Frontend dev server
    "http://localhost:8000",  # Backend
    "http://127.0.0.1:3000",
    "http://127.0.0.1:8000",
]

# Add production URLs from environment
if prod_url := os.getenv("FRONTEND_URL"):
    origins.append(prod_url)

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Middleware for request logging
@app.middleware("http")
async def log_requests(request: Request, call_next):
    """Log all incoming requests."""
    start_time = time.time()

    # Log request
    logger.info(f"Request: {request.method} {request.url.path}")

    # Process request
    response = await call_next(request)

    # Calculate duration
    duration = (time.time() - start_time) * 1000  # Convert to ms

    # Log response
    logger.info(f"Response: {response.status_code} | Duration: {duration:.2f}ms")

    return response


@app.get("/", tags=["Health"])
async def root():
    """Root endpoint - API information."""
    return {
        "message": "Physical AI & Humanoid Robotics RAG Chatbot API",
        "version": "1.0.0",
        "endpoints": {
            "health": "/health",
            "chat": "/chat",
            "feedback": "/feedback"
        }
    }


@app.get("/health", response_model=HealthResponse, tags=["Health"])
async def health_check(db: Session = Depends(get_db)):
    """
    Health check endpoint.
    Returns status of database and Qdrant connections.
    """
    try:
        # Check database
        db.execute("SELECT 1")
        db_status = "connected"
    except Exception as e:
        logger.error(f"Database health check failed: {e}")
        db_status = f"error: {str(e)}"

    # Check Qdrant
    try:
        if rag_agent and rag_agent.retriever.handle_connection_failures():
            qdrant_status = "connected"
        else:
            qdrant_status = "disconnected"
    except Exception as e:
        logger.error(f"Qdrant health check failed: {e}")
        qdrant_status = f"error: {str(e)}"

    return HealthResponse(
        status="healthy" if db_status == "connected" and qdrant_status == "connected" else "degraded",
        timestamp=datetime.utcnow(),
        database=db_status,
        qdrant=qdrant_status,
        version="1.0.0"
    )


@app.post("/chat", response_model=ChatResponse, tags=["Chat"])
@limiter.limit("30/minute")  # Rate limit: 30 requests per minute
async def chat_endpoint(
    request: Request,
    chat_request: ChatRequest,
    db: Session = Depends(get_db)
):
    """
    Chat endpoint - Process user queries and return AI responses.

    - **session_id**: Unique session identifier
    - **query_text**: User's question
    - **selected_text**: Optional text selected by user for context
    - **collection_name**: Qdrant collection to search

    Returns AI-generated response with source citations.
    """
    start_time = time.time()

    try:
        # Get or create session
        session = db.query(ChatSession).filter(
            ChatSession.session_id == chat_request.session_id
        ).first()

        if not session:
            session = ChatSession(session_id=chat_request.session_id)
            db.add(session)
            db.commit()
            db.refresh(session)
            logger.info(f"Created new session: {chat_request.session_id}")

        # Store user query
        user_query = UserQuery(
            session_id=session.id,
            query_text=chat_request.query_text,
            selected_text=chat_request.selected_text,
            collection_name=chat_request.collection_name
        )
        db.add(user_query)
        db.commit()
        db.refresh(user_query)
        logger.info(f"Stored user query: {user_query.id}")

        # Query RAG Agent
        if not rag_agent:
            raise HTTPException(status_code=503, detail="RAG Agent not initialized")

        try:
            # Build query with selected text context if provided
            full_query = chat_request.query_text
            if chat_request.selected_text:
                full_query = f"Context: {chat_request.selected_text}\n\nQuestion: {chat_request.query_text}"

            # Get response from RAG agent
            agent_response = rag_agent.query(full_query, top_k=5)

            # Extract response data
            response_text = agent_response.get("response", "I couldn't generate a response.")
            retrieved_chunks = agent_response.get("retrieved_chunks", [])

            # Calculate metrics
            avg_relevance = 0.0
            if retrieved_chunks:
                avg_relevance = sum(c.get("relevance_score", 0.0) for c in retrieved_chunks) / len(retrieved_chunks)

            # Extract source URLs
            sources = []
            for chunk in retrieved_chunks:
                source_url = chunk.get("source_url", "")
                if source_url and source_url not in sources:
                    sources.append(source_url)

        except Exception as e:
            logger.error(f"Error querying RAG agent: {e}")
            raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

        # Store chatbot response
        response_time = (time.time() - start_time) * 1000  # Convert to ms

        chatbot_response = ChatbotResponse(
            session_id=session.id,
            query_id=user_query.id,
            response_text=response_text,
            sources=sources,
            retrieved_chunks_count=len(retrieved_chunks),
            avg_relevance_score=avg_relevance,
            response_time_ms=response_time
        )
        db.add(chatbot_response)
        db.commit()
        db.refresh(chatbot_response)
        logger.info(f"Stored chatbot response: {chatbot_response.id}")

        return ChatResponse(
            response_text=response_text,
            response_id=chatbot_response.id,
            sources=sources,
            retrieved_chunks_count=len(retrieved_chunks),
            avg_relevance_score=avg_relevance,
            response_time_ms=response_time
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error in chat endpoint: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@app.post("/feedback", response_model=FeedbackResponse, tags=["Chat"])
@limiter.limit("60/minute")  # Rate limit: 60 requests per minute
async def feedback_endpoint(
    request: Request,
    feedback_request: FeedbackRequest,
    db: Session = Depends(get_db)
):
    """
    Feedback endpoint - Store user feedback on chatbot responses.

    - **response_id**: ID of the response to provide feedback for
    - **feedback**: 1 for positive (👍), -1 for negative (👎)

    Returns confirmation of stored feedback.
    """
    try:
        # Verify response exists
        response = db.query(ChatbotResponse).filter(
            ChatbotResponse.id == feedback_request.response_id
        ).first()

        if not response:
            raise HTTPException(status_code=404, detail="Response not found")

        # Check if feedback already exists
        existing_feedback = db.query(UserFeedback).filter(
            UserFeedback.response_id == feedback_request.response_id
        ).first()

        if existing_feedback:
            # Update existing feedback
            existing_feedback.feedback = feedback_request.feedback
            existing_feedback.created_at = datetime.utcnow()
            db.commit()
            db.refresh(existing_feedback)
            logger.info(f"Updated feedback for response {feedback_request.response_id}")

            return FeedbackResponse(
                message="Feedback updated successfully",
                feedback_id=existing_feedback.id
            )
        else:
            # Create new feedback
            user_feedback = UserFeedback(
                response_id=feedback_request.response_id,
                feedback=feedback_request.feedback
            )
            db.add(user_feedback)
            db.commit()
            db.refresh(user_feedback)
            logger.info(f"Stored new feedback for response {feedback_request.response_id}")

            return FeedbackResponse(
                message="Feedback submitted successfully",
                feedback_id=user_feedback.id
            )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error storing feedback: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Error storing feedback: {str(e)}")


# Error handlers
@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """Handle HTTP exceptions."""
    return JSONResponse(
        status_code=exc.status_code,
        content=ErrorResponse(
            error=exc.detail,
            detail=None,
            status_code=exc.status_code
        ).model_dump()
    )


@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """Handle general exceptions."""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content=ErrorResponse(
            error="Internal Server Error",
            detail=str(exc),
            status_code=500
        ).model_dump()
    )


if __name__ == "__main__":
    import uvicorn

    port = int(os.getenv("PORT", "8000"))
    host = os.getenv("HOST", "0.0.0.0")

    logger.info(f"Starting server on {host}:{port}")

    uvicorn.run(
        "main:app",
        host=host,
        port=port,
        reload=True,  # Enable auto-reload for development
        log_level="info"
    )
