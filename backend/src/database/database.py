"""
Database connection and session management for PostgreSQL.
"""

import os
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.pool import NullPool
from contextlib import contextmanager
from typing import Generator
import logging

from .models import Base

logger = logging.getLogger(__name__)


class DatabaseManager:
    """Manages database connections and sessions."""

    def __init__(self):
        """Initialize database connection."""
        # Get database URL from environment
        self.database_url = os.getenv("DATABASE_URL")

        if not self.database_url:
            logger.warning("DATABASE_URL not set, using in-memory SQLite for development")
            self.database_url = "sqlite:///./chat_history.db"

        # Handle Neon/Railway postgres URL format (postgres:// -> postgresql://)
        if self.database_url.startswith("postgres://"):
            self.database_url = self.database_url.replace("postgres://", "postgresql://", 1)

        # Create engine with appropriate settings
        if "sqlite" in self.database_url:
            # SQLite configuration
            self.engine = create_engine(
                self.database_url,
                connect_args={"check_same_thread": False},
                echo=False
            )
        else:
            # PostgreSQL configuration for production
            self.engine = create_engine(
                self.database_url,
                poolclass=NullPool,  # Serverless-friendly: no connection pooling
                echo=False,
                pool_pre_ping=True,  # Verify connections before using
            )

        # Create session factory
        self.SessionLocal = sessionmaker(
            autocommit=False,
            autoflush=False,
            bind=self.engine
        )

        logger.info(f"Database initialized with URL: {self.database_url.split('@')[0]}@...")

    def create_tables(self):
        """Create all database tables."""
        try:
            Base.metadata.create_all(bind=self.engine)
            logger.info("Database tables created successfully")
        except Exception as e:
            logger.error(f"Error creating database tables: {e}")
            raise

    def drop_tables(self):
        """Drop all database tables (use with caution!)."""
        try:
            Base.metadata.drop_all(bind=self.engine)
            logger.info("Database tables dropped successfully")
        except Exception as e:
            logger.error(f"Error dropping database tables: {e}")
            raise

    @contextmanager
    def get_session(self) -> Generator[Session, None, None]:
        """
        Provide a transactional scope for database operations.

        Usage:
            with db_manager.get_session() as session:
                session.add(obj)
                session.commit()
        """
        session = self.SessionLocal()
        try:
            yield session
            session.commit()
        except Exception as e:
            session.rollback()
            logger.error(f"Database session error: {e}")
            raise
        finally:
            session.close()

    def get_db(self) -> Generator[Session, None, None]:
        """
        Dependency for FastAPI to get database session.

        Usage in FastAPI:
            @app.get("/items")
            def get_items(db: Session = Depends(db_manager.get_db)):
                ...
        """
        session = self.SessionLocal()
        try:
            yield session
        finally:
            session.close()


# Global database manager instance
db_manager = DatabaseManager()


def init_database():
    """Initialize database tables."""
    db_manager.create_tables()


def get_db() -> Generator[Session, None, None]:
    """FastAPI dependency to get database session."""
    return db_manager.get_db()
