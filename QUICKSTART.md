# 🚀 Quick Start Guide - 5 Minutes to Launch

## Step 1: Configure Environment (1 min)

```bash
# Copy environment template
copy .env.example .env

# Edit .env with your API keys
notepad .env
```

**Required values:**
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://your-qdrant-instance.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
Gemini_Api_Key=your_gemini_api_key_here
```

**Where to get API keys:**
- Cohere: https://cohere.com (free tier available)
- Qdrant: https://cloud.qdrant.io (free tier available)
- Gemini: https://aistudio.google.com (free tier available)

---

## Step 2: Install Dependencies (2 min)

```bash
# Backend dependencies
pip install -r backend/requirements.txt

# Frontend dependencies
cd frontend
npm install
cd ..
```

---

## Step 3: Initialize Database (30 seconds)

```bash
python init_database.py
```

This creates:
- `chat_sessions` table
- `user_queries` table
- `chatbot_responses` table
- `user_feedback` table

---

## Step 4: Verify Setup (30 seconds)

```bash
python test_setup.py
```

This checks:
- ✅ Environment variables configured
- ✅ Dependencies installed
- ✅ Database connection
- ✅ Qdrant connection
- ⚠️ If Qdrant collection doesn't exist, run ingestion (Step 5)

---

## Step 5: Ingest Content (Optional - 1 min)

**Only needed if you haven't ingested content yet:**

```bash
# Ingest from your documentation website
python ingest_robotics_content.py https://your-docs-url.com
```

This will:
1. Scrape content from your docs
2. Chunk text into 512-token segments
3. Generate embeddings with Cohere
4. Store in Qdrant vector database

---

## Step 6: Launch! (10 seconds)

```bash
start_dev.bat
```

This starts:
- 🔧 Backend API server on http://localhost:8000
- 🎨 Frontend app on http://localhost:3000

**Wait 10 seconds** for both servers to start.

---

## Step 7: Use the Chatbot

1. **Open browser:** http://localhost:3000
2. **Click the chat icon** (bottom-right corner)
3. **Ask a question** about your content
4. **Get AI-powered answers** with source citations!

---

## 🎯 Example Questions to Try

```
What is reinforcement learning?
Explain the difference between supervised and unsupervised learning
How do humanoid robots work?
What are the key components of a robot?
```

---

## 📊 Monitor Your System

- **API Docs:** http://localhost:8000/docs
- **Health Check:** http://localhost:8000/health
- **Database:** SQLite file `chat_history.db` (default)

---

## 🐛 Troubleshooting

### Backend won't start
```bash
# Check if port 8000 is in use
netstat -ano | findstr :8000

# Kill the process
taskkill /PID <PID> /F
```

### Frontend won't start
```bash
# Check if port 3000 is in use
netstat -ano | findstr :3000

# Kill the process
taskkill /PID <PID> /F
```

### Chatbot returns no results
- ✅ Verify Qdrant connection in http://localhost:8000/health
- ✅ Run ingestion if collection is empty
- ✅ Check QDRANT_URL and QDRANT_API_KEY in .env

### Database errors
```bash
# Reinitialize database
python init_database.py
```

---

## 🎓 Next Steps

Once everything works:

1. **Read SETUP.md** for detailed documentation
2. **Customize the chatbot** (system prompts in `agent.py`)
3. **Deploy to production** (see SETUP.md#production-deployment)
4. **Add analytics** (Sentry, PostHog, etc.)

---

## 📞 Need Help?

- Check SETUP.md for detailed troubleshooting
- Review API logs in the console
- Test individual components with `test_setup.py`

---

**You're all set! Happy building! 🤖**
