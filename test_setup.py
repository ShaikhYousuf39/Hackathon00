#!/usr/bin/env python3
"""
Quick setup verification script.
Tests that all components are properly configured.
"""

import os
import sys
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))


def check_env_file():
    """Check if .env file exists and has required keys."""
    print("🔍 Checking environment configuration...")

    env_path = project_root / ".env"
    if not env_path.exists():
        print("  ❌ .env file not found!")
        print("  → Copy .env.example to .env and configure your API keys")
        return False

    # Load and check required keys
    from dotenv import load_dotenv
    load_dotenv()

    required_keys = [
        "COHERE_API_KEY",
        "QDRANT_URL",
        "QDRANT_API_KEY",
        "Gemini_Api_Key"
    ]

    missing_keys = []
    for key in required_keys:
        value = os.getenv(key)
        if not value or value == f"your_{key.lower()}_here":
            missing_keys.append(key)

    if missing_keys:
        print(f"  ⚠️  Missing or unconfigured keys: {', '.join(missing_keys)}")
        return False

    print("  ✅ Environment configuration OK")
    return True


def check_dependencies():
    """Check if required Python packages are installed."""
    print("\n🔍 Checking Python dependencies...")

    required_packages = {
        "fastapi": "FastAPI",
        "uvicorn": "Uvicorn",
        "sqlalchemy": "SQLAlchemy",
        "qdrant_client": "Qdrant Client",
        "cohere": "Cohere",
        "pydantic": "Pydantic",
        "slowapi": "SlowAPI",
    }

    missing = []
    for package, name in required_packages.items():
        try:
            __import__(package)
            print(f"  ✅ {name}")
        except ImportError:
            print(f"  ❌ {name} - MISSING")
            missing.append(package)

    if missing:
        print(f"\n  Install missing packages:")
        print(f"  pip install {' '.join(missing)}")
        return False

    return True


def check_project_structure():
    """Check if all required files and directories exist."""
    print("\n🔍 Checking project structure...")

    required_paths = [
        "backend/src/api/main.py",
        "backend/src/api/schemas.py",
        "backend/src/database/models.py",
        "backend/src/database/database.py",
        "agent.py",
        "retriever.py",
        "config.py",
        "models.py",
        "frontend/src/components/Chat.js",
        "frontend/src/components/FloatingChatIcon.js",
    ]

    all_exist = True
    for path in required_paths:
        full_path = project_root / path
        if full_path.exists():
            print(f"  ✅ {path}")
        else:
            print(f"  ❌ {path} - MISSING")
            all_exist = False

    return all_exist


def check_qdrant_connection():
    """Check if Qdrant connection works."""
    print("\n🔍 Checking Qdrant connection...")

    try:
        from retriever import RAGRetriever

        retriever = RAGRetriever()
        if retriever.handle_connection_failures():
            print("  ✅ Qdrant connection successful")

            # Check if collection exists
            try:
                collection_info = retriever.qdrant_client.get_collection(retriever.collection_name)
                print(f"  ✅ Collection '{retriever.collection_name}' exists")
                print(f"     Vectors count: {collection_info.vectors_count}")
                return True
            except Exception as e:
                print(f"  ⚠️  Collection '{retriever.collection_name}' not found")
                print(f"     Run ingestion script to populate the collection")
                return False
        else:
            print("  ❌ Qdrant connection failed")
            return False
    except Exception as e:
        print(f"  ❌ Error: {e}")
        return False


def check_database():
    """Check if database can be initialized."""
    print("\n🔍 Checking database setup...")

    try:
        from backend.src.database.database import db_manager

        # Try to create session
        with db_manager.get_session() as session:
            # Simple query test
            session.execute("SELECT 1")

        print("  ✅ Database connection successful")
        return True
    except Exception as e:
        print(f"  ⚠️  Database test failed: {e}")
        print(f"     Run: python init_database.py")
        return False


def main():
    """Run all checks."""
    print("=" * 60)
    print("  RAG Chatbot Setup Verification")
    print("=" * 60)
    print()

    checks = [
        ("Environment Configuration", check_env_file),
        ("Python Dependencies", check_dependencies),
        ("Project Structure", check_project_structure),
        ("Database", check_database),
        ("Qdrant Vector DB", check_qdrant_connection),
    ]

    results = {}
    for name, check_func in checks:
        try:
            results[name] = check_func()
        except Exception as e:
            print(f"\n❌ Error in {name}: {e}")
            results[name] = False

    print("\n" + "=" * 60)
    print("  Summary")
    print("=" * 60)

    all_passed = all(results.values())

    for name, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"  {status} - {name}")

    print()

    if all_passed:
        print("🎉 All checks passed! Your setup is ready.")
        print()
        print("Next steps:")
        print("  1. Initialize database: python init_database.py")
        print("  2. Start servers: start_dev.bat")
        print("  3. Open browser: http://localhost:3000")
    else:
        print("⚠️  Some checks failed. Please fix the issues above.")
        print()
        print("Common fixes:")
        print("  1. Configure .env file with your API keys")
        print("  2. Install dependencies: pip install -r backend/requirements.txt")
        print("  3. Initialize database: python init_database.py")
        print("  4. Run ingestion: python ingest_robotics_content.py <url>")

    print()
    print("=" * 60)

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
