# 🤖 Physical AI & Humanoid Robotics RAG Chatbot - Setup Guide

## 📋 Table of Contents
- [Overview](#overview)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Detailed Setup](#detailed-setup)
- [Running the Application](#running-the-application)
- [API Documentation](#api-documentation)
- [Troubleshooting](#troubleshooting)
- [Production Deployment](#production-deployment)

---

## 🎯 Overview

This is a production-ready RAG (Retrieval-Augmented Generation) chatbot system integrated into a Docusaurus-based educational platform for Physical AI and Humanoid Robotics.

### **Key Features**
- ✅ **Frontend**: React-based chat UI with floating icon, source citations, feedback system
- ✅ **Backend**: FastAPI server with RAG pipeline (Qdrant + Gemini)
- ✅ **Database**: PostgreSQL/SQLite for chat history and analytics
- ✅ **Rate Limiting**: Protection against abuse (30 req/min for chat, 60 req/min for feedback)
- ✅ **Security**: Input validation, SQL injection prevention, XSS protection
- ✅ **Monitoring**: Comprehensive logging and health checks

---

## 🏗️ Architecture

```
┌─────────────────┐
│  Frontend       │
│  (React/Docusaurus)
│  Port: 3000     │
└────────┬────────┘
         │ HTTP
         ▼
┌─────────────────┐      ┌──────────────┐
│  Backend API    │──────│  Qdrant DB   │
│  (FastAPI)      │      │  (Vectors)   │
│  Port: 8000     │      └──────────────┘
└────────┬────────┘
         │                ┌──────────────┐
         │────────────────│  PostgreSQL  │
         │                │  (Sessions)  │
         │                └──────────────┘
         │
         │                ┌──────────────┐
         └────────────────│  Gemini API  │
                          │  (LLM)       │
                          └──────────────┘
```

### **Tech Stack**
- **Frontend**: React 19, Docusaurus 3.9
- **Backend**: FastAPI, Python 3.10+
- **Vector DB**: Qdrant Cloud
- **LLM**: Google Gemini 2.5 Flash
- **Embeddings**: Cohere embed-english-v3.0
- **Database**: PostgreSQL (Neon) / SQLite (dev)
- **ORM**: SQLAlchemy 2.0

---

## 📦 Prerequisites

### **Required**
- **Node.js** 18-20 (for frontend)
- **Python** 3.10+ (for backend)
- **Git** (for version control)

### **API Keys Required**
1. **Cohere API Key** - Get from [cohere.com](https://cohere.com)
2. **Qdrant Cloud** - Get from [cloud.qdrant.io](https://cloud.qdrant.io)
3. **Gemini API Key** - Get from [Google AI Studio](https://aistudio.google.com)
4. **Database URL** (Optional for production):
   - [Neon](https://neon.tech) (recommended)
   - [Railway](https://railway.app)
   - [Supabase](https://supabase.com)

---

## ⚡ Quick Start

### **1. Clone and Navigate**
```bash
cd f:\Miss Hackthon\learn_humanoid_robot
```

### **2. Configure Environment**
```bash
# Copy the example environment file
copy .env.example .env

# Edit .env with your API keys
notepad .env
```

**Required values in `.env`:**
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://your-qdrant-instance.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
Gemini_Api_Key=your_gemini_api_key_here

# Optional: Leave empty to use SQLite
DATABASE_URL=postgresql://user:password@host:5432/chatbot_db
```

### **3. Initialize Database**
```bash
python init_database.py
```

### **4. Start Development Servers**
```bash
# Option 1: Start both frontend and backend (recommended)
start_dev.bat

# Option 2: Start backend only
start_backend.bat
```

### **5. Access the Application**
- **Frontend**: http://localhost:3000
- **Backend API**: http://localhost:8000
- **API Docs**: http://localhost:8000/docs

---

## 🔧 Detailed Setup

### **Step 1: Install Backend Dependencies**
```bash
pip install -r backend/requirements.txt
```

**Key dependencies:**
- `fastapi` - Web framework
- `uvicorn` - ASGI server
- `sqlalchemy` - ORM
- `qdrant-client` - Vector database
- `cohere` - Embeddings API
- `slowapi` - Rate limiting

### **Step 2: Install Frontend Dependencies**
```bash
cd frontend
npm install
cd ..
```

### **Step 3: Ingest Content into Qdrant**

If you haven't ingested your content yet:

```bash
# Ingest website content
python ingest_robotics_content.py https://your-docs-url.com

# Or use full ingestion script
python ingest_full_robotics_content.py
```

This will:
1. Scrape content from your documentation
2. Chunk text into 512-token segments
3. Generate embeddings with Cohere
4. Store vectors in Qdrant

### **Step 4: Verify Setup**

Test the RAG agent:
```bash
python agent.py --interactive
```

Test the backend API:
```bash
python backend/src/api/main.py
```

Visit http://localhost:8000/health to check system status.

---

## 🚀 Running the Application

### **Development Mode**

**Option 1: Automated (Recommended)**
```bash
start_dev.bat
```
This starts both servers in separate windows.

**Option 2: Manual**

Terminal 1 (Backend):
```bash
cd backend/src/api
python main.py
```

Terminal 2 (Frontend):
```bash
cd frontend
npm start
```

### **Production Mode**

1. **Build frontend:**
```bash
cd frontend
npm run build
cd ..
```

2. **Set environment variables:**
```bash
set PRODUCTION=true
set DATABASE_URL=postgresql://...
```

3. **Run backend with production settings:**
```bash
python backend/src/api/main.py
```

Or use a production ASGI server:
```bash
uvicorn backend.src.api.main:app --host 0.0.0.0 --port 8000 --workers 4
```

---

## 📚 API Documentation

### **Endpoints**

#### **1. POST /chat**
Process user queries and return AI responses.

**Request:**
```json
{
  "session_id": "user-123",
  "query_text": "What is reinforcement learning?",
  "selected_text": null,
  "collection_name": "document_embeddings"
}
```

**Response:**
```json
{
  "response_text": "Reinforcement learning is a type of machine learning...",
  "response_id": "abc-123-def",
  "sources": [
    "https://example.com/chapter1#reinforcement-learning"
  ],
  "retrieved_chunks_count": 5,
  "avg_relevance_score": 0.85,
  "response_time_ms": 1234.56
}
```

#### **2. POST /feedback**
Submit user feedback on responses.

**Request:**
```json
{
  "response_id": "abc-123-def",
  "feedback": 1
}
```

**Response:**
```json
{
  "message": "Feedback submitted successfully",
  "feedback_id": "xyz-789"
}
```

#### **3. GET /health**
Check system health.

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2024-01-01T00:00:00Z",
  "database": "connected",
  "qdrant": "connected",
  "version": "1.0.0"
}
```

### **Interactive API Docs**
Visit http://localhost:8000/docs for Swagger UI documentation.

---

## 🐛 Troubleshooting

### **Common Issues**

#### **1. Database Connection Error**
```
Error: DATABASE_URL not set
```
**Solution**:
- For development: Leave `DATABASE_URL` empty in `.env` (uses SQLite)
- For production: Set valid PostgreSQL URL

#### **2. Qdrant Connection Failed**
```
Error: Qdrant connection failed
```
**Solution**:
- Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Check Qdrant Cloud status
- Ensure collection `document_embeddings` exists (run ingestion first)

#### **3. Port Already in Use**
```
Error: Address already in use
```
**Solution**:
```bash
# Find process using port 8000
netstat -ano | findstr :8000

# Kill process (replace PID)
taskkill /PID <PID> /F
```

#### **4. Frontend Can't Connect to Backend**
```
Failed to fetch response from chatbot
```
**Solution**:
- Ensure backend is running on http://localhost:8000
- Check CORS settings in `backend/src/api/main.py`
- Verify frontend proxy in `frontend/package.json`

#### **5. Import Errors**
```
ModuleNotFoundError: No module named 'agents'
```
**Solution**:
```bash
pip install openai-agents qdrant-client cohere python-dotenv
```

---

## 🌐 Production Deployment

### **Environment Variables for Production**

```env
# Required
COHERE_API_KEY=prod_cohere_key
QDRANT_URL=https://prod-qdrant.cloud.qdrant.io
QDRANT_API_KEY=prod_qdrant_key
Gemini_Api_Key=prod_gemini_key
DATABASE_URL=postgresql://user:pass@host:5432/db

# Server
HOST=0.0.0.0
PORT=8000
FRONTEND_URL=https://your-frontend-domain.com
PRODUCTION=true

# Optional
SENTRY_DSN=your_sentry_dsn
LOG_LEVEL=WARNING
```

### **Deployment Checklist**

- [ ] All API keys configured in production `.env`
- [ ] Database initialized (`python init_database.py`)
- [ ] Content ingested into Qdrant
- [ ] Frontend built (`npm run build`)
- [ ] CORS origins updated for production domain
- [ ] Rate limiting configured
- [ ] Logging/monitoring set up (Sentry, etc.)
- [ ] Health check endpoint tested
- [ ] SSL/TLS certificates configured
- [ ] Environment variables secured (not in git)

### **Recommended Platforms**

**Backend:**
- Railway (recommended for simplicity)
- Render
- Fly.io
- AWS EC2 / Google Cloud Run

**Frontend:**
- Vercel (recommended for Docusaurus)
- Netlify
- Cloudflare Pages

**Database:**
- Neon (recommended - serverless Postgres)
- Supabase
- Railway Postgres

---

## 📊 Database Schema

### **Tables**

1. **chat_sessions** - User conversation sessions
2. **user_queries** - All user questions
3. **chatbot_responses** - AI responses with sources
4. **user_feedback** - Thumbs up/down ratings

### **View Schema**
```bash
python init_database.py
```

---

## 🔒 Security Features

- ✅ **Input Validation**: Pydantic schemas for all requests
- ✅ **Rate Limiting**: 30 req/min (chat), 60 req/min (feedback)
- ✅ **SQL Injection Prevention**: SQLAlchemy ORM
- ✅ **XSS Protection**: Query validation and sanitization
- ✅ **CORS**: Restricted to allowed origins
- ✅ **Error Handling**: Graceful degradation, no data leaks

---

## 📝 Development Notes

### **Project Structure**
```
learn_humanoid_robot/
├── frontend/              # Docusaurus + React UI
│   └── src/components/    # Chat.js, FloatingChatIcon.js
├── backend/
│   ├── src/
│   │   ├── api/           # FastAPI server (main.py)
│   │   ├── database/      # SQLAlchemy models
│   │   ├── scraper.py     # Content scraping
│   │   ├── chunker.py     # Text chunking
│   │   ├── embeddings.py  # Cohere integration
│   │   └── storage.py     # Qdrant storage
├── agent.py               # RAG Agent (Gemini)
├── retriever.py           # Qdrant retrieval
├── config.py              # Configuration
├── models.py              # Data models
├── init_database.py       # DB initialization
└── .env.example           # Environment template
```

### **Testing**

**Backend tests:**
```bash
cd backend
pytest
```

**Frontend tests:**
```bash
cd frontend
npm test
```

---

## 🤝 Support

For issues or questions:
1. Check [Troubleshooting](#troubleshooting) section
2. Review API logs in console
3. Check `/health` endpoint
4. Review GitHub issues (if applicable)

---

## 📄 License

See project LICENSE file for details.

---

**Built with ❤️ for Physical AI & Humanoid Robotics Education**
