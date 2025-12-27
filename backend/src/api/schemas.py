"""
Pydantic schemas for API request/response validation.
"""

from pydantic import BaseModel, Field, field_validator
from typing import Optional, List, Dict, Any
from datetime import datetime


class ChatRequest(BaseModel):
    """Request schema for chat endpoint."""
    session_id: str = Field(..., description="Unique session identifier")
    query_text: str = Field(..., min_length=1, max_length=1000, description="User query text")
    selected_text: Optional[str] = Field(None, description="Text selected by user for context")
    collection_name: Optional[str] = Field("document_embeddings", description="Qdrant collection name")

    @field_validator('query_text')
    @classmethod
    def validate_query_text(cls, v):
        if not v or not v.strip():
            raise ValueError('Query text cannot be empty')
        return v.strip()

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "session_id": "123",
                    "query_text": "What is reinforcement learning?",
                    "selected_text": None,
                    "collection_name": "document_embeddings"
                }
            ]
        }
    }


class Source(BaseModel):
    """Schema for source citation."""
    url: str
    section: Optional[str] = None
    heading: Optional[str] = None
    content_preview: Optional[str] = None
    relevance_score: Optional[float] = None


class ChatResponse(BaseModel):
    """Response schema for chat endpoint."""
    response_text: str = Field(..., description="Chatbot response text")
    response_id: str = Field(..., description="Unique response identifier")
    sources: List[str] = Field(default=[], description="Source URLs or content previews")
    retrieved_chunks_count: int = Field(default=0, description="Number of chunks retrieved")
    avg_relevance_score: Optional[float] = Field(None, description="Average relevance score")
    response_time_ms: Optional[float] = Field(None, description="Response time in milliseconds")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "response_text": "Reinforcement learning is a type of machine learning...",
                    "response_id": "abc-123-def",
                    "sources": [
                        "https://example.com/chapter1#reinforcement-learning",
                        "https://example.com/chapter3#rl-applications"
                    ],
                    "retrieved_chunks_count": 5,
                    "avg_relevance_score": 0.85,
                    "response_time_ms": 1234.56
                }
            ]
        }
    }


class FeedbackRequest(BaseModel):
    """Request schema for feedback endpoint."""
    response_id: str = Field(..., description="Response ID to provide feedback for")
    feedback: int = Field(..., ge=-1, le=1, description="Feedback value: 1 (positive), -1 (negative)")

    @field_validator('feedback')
    @classmethod
    def validate_feedback(cls, v):
        if v not in [1, -1]:
            raise ValueError('Feedback must be 1 (positive) or -1 (negative)')
        return v

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "response_id": "abc-123-def",
                    "feedback": 1
                }
            ]
        }
    }


class FeedbackResponse(BaseModel):
    """Response schema for feedback endpoint."""
    message: str
    feedback_id: str


class HealthResponse(BaseModel):
    """Response schema for health check endpoint."""
    status: str
    timestamp: datetime
    database: str
    qdrant: str
    version: str

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "status": "healthy",
                    "timestamp": "2024-01-01T00:00:00Z",
                    "database": "connected",
                    "qdrant": "connected",
                    "version": "1.0.0"
                }
            ]
        }
    }


class ErrorResponse(BaseModel):
    """Response schema for errors."""
    error: str
    detail: Optional[str] = None
    status_code: int

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "error": "Validation Error",
                    "detail": "Query text cannot be empty",
                    "status_code": 422
                }
            ]
        }
    }
