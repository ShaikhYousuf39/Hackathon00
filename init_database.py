#!/usr/bin/env python3
"""
Database initialization script.
Creates all tables and optionally seeds test data.
"""

import os
import sys
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from backend.src.database.database import db_manager, init_database
from backend.src.database.models import ChatSession, UserQuery, ChatbotResponse, UserFeedback
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


def main():
    """Initialize database tables."""
    print("=" * 60)
    print("  Database Initialization")
    print("=" * 60)
    print()

    # Check if DATABASE_URL is set
    database_url = os.getenv("DATABASE_URL")
    if not database_url:
        print("⚠️  WARNING: DATABASE_URL not set")
        print("Using SQLite database for development: chat_history.db")
        print()
    else:
        # Mask credentials for display
        masked_url = database_url.split("@")[-1] if "@" in database_url else database_url
        print(f"📊 Database: {masked_url}")
        print()

    try:
        # Initialize database (creates all tables)
        print("Creating database tables...")
        init_database()

        print()
        print("✅ Database initialized successfully!")
        print()
        print("Tables created:")
        print("  - chat_sessions")
        print("  - user_queries")
        print("  - chatbot_responses")
        print("  - user_feedback")
        print()

        # Verify tables were created
        with db_manager.get_session() as session:
            # Try to query each table
            session.query(ChatSession).count()
            session.query(UserQuery).count()
            session.query(ChatbotResponse).count()
            session.query(UserFeedback).count()

        print("✅ All tables verified!")
        print()
        print("=" * 60)
        print("  Database is ready to use!")
        print("=" * 60)

    except Exception as e:
        print()
        print(f"❌ Error initializing database: {e}")
        print()
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
