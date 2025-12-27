"""
SQLAlchemy database models for chat history, sessions, and feedback.
"""

from datetime import datetime
from sqlalchemy import Column, Integer, String, Text, DateTime, Float, ForeignKey, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
import uuid

Base = declarative_base()


class ChatSession(Base):
    """Represents a user chat session."""
    __tablename__ = "chat_sessions"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    session_id = Column(String(100), unique=True, nullable=False, index=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
    metadata = Column(JSON, default={})

    # Relationships
    queries = relationship("UserQuery", back_populates="session", cascade="all, delete-orphan")
    responses = relationship("ChatbotResponse", back_populates="session", cascade="all, delete-orphan")

    def __repr__(self):
        return f"<ChatSession(session_id={self.session_id}, created_at={self.created_at})>"


class UserQuery(Base):
    """Represents a user query in a chat session."""
    __tablename__ = "user_queries"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    session_id = Column(String(36), ForeignKey("chat_sessions.id"), nullable=False)
    query_text = Column(Text, nullable=False)
    selected_text = Column(Text, nullable=True)  # Text highlighted by user
    collection_name = Column(String(100), nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    metadata = Column(JSON, default={})

    # Relationships
    session = relationship("ChatSession", back_populates="queries")
    response = relationship("ChatbotResponse", back_populates="query", uselist=False)

    def __repr__(self):
        return f"<UserQuery(id={self.id}, query_text={self.query_text[:50]}...)>"


class ChatbotResponse(Base):
    """Represents a chatbot response to a user query."""
    __tablename__ = "chatbot_responses"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    session_id = Column(String(36), ForeignKey("chat_sessions.id"), nullable=False)
    query_id = Column(String(36), ForeignKey("user_queries.id"), nullable=False)
    response_text = Column(Text, nullable=False)
    sources = Column(JSON, default=[])  # List of source URLs/chunks
    retrieved_chunks_count = Column(Integer, default=0)
    avg_relevance_score = Column(Float, nullable=True)
    response_time_ms = Column(Float, nullable=True)  # Response time in milliseconds
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    metadata = Column(JSON, default={})

    # Relationships
    session = relationship("ChatSession", back_populates="responses")
    query = relationship("UserQuery", back_populates="response")
    feedback = relationship("UserFeedback", back_populates="response", uselist=False)

    def __repr__(self):
        return f"<ChatbotResponse(id={self.id}, response_text={self.response_text[:50]}...)>"


class UserFeedback(Base):
    """Represents user feedback on a chatbot response."""
    __tablename__ = "user_feedback"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    response_id = Column(String(36), ForeignKey("chatbot_responses.id"), nullable=False, unique=True)
    feedback = Column(Integer, nullable=False)  # 1 for positive, -1 for negative
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    metadata = Column(JSON, default={})

    # Relationships
    response = relationship("ChatbotResponse", back_populates="feedback")

    def __repr__(self):
        return f"<UserFeedback(response_id={self.response_id}, feedback={self.feedback})>"
