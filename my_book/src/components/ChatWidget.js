import React, { useState, useRef, useEffect } from 'react';
import './ChatWidget.css';
import { authClient } from '../lib/auth-client';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isRecording, setIsRecording] = useState(false);
  const [session, setSession] = useState(null);

  const recognitionRef = useRef(null);
  const messagesEndRef = useRef(null);

  useEffect(() => {
    // Check session on mount just to get user name if available
    const checkSession = async () => {
      const { data } = await authClient.getSession();
      setSession(data);
    };
    checkSession();
  }, []);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    if ('webkitSpeechRecognition' in window || 'SpeechRecognition' in window) {
      const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
      recognitionRef.current = new SpeechRecognition();
      recognitionRef.current.continuous = false;
      recognitionRef.current.interimResults = false;

      recognitionRef.current.onresult = (event) => {
        const transcript = event.results[0][0].transcript;
        setInput((prev) => prev + (prev ? ' ' : '') + transcript);
        setIsRecording(false);
      };

      recognitionRef.current.onerror = (event) => {
        console.error('Speech recognition error', event.error);
        setIsRecording(false);
      };

      recognitionRef.current.onend = () => {
        setIsRecording(false);
      };
    }
  }, []);

  const startRecording = () => {
    if (recognitionRef.current) {
      try {
        recognitionRef.current.start();
        setIsRecording(true);
      } catch (error) {
        console.error("Error starting speech recognition:", error);
      }
    } else {
      alert("Speech recognition is not supported in this browser.");
    }
  };

  const stopRecording = () => {
    if (recognitionRef.current) {
      recognitionRef.current.stop();
      setIsRecording(false);
    }
  };

  const handleLogout = async () => {
    await authClient.signOut();
    setSession(null);
    // Optionally redirect or show a message
    window.location.reload();
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!input.trim()) return;

    const userMessage = { text: input, sender: 'user' };
    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    const API_URL = "https://backend-nine-black-50.vercel.app/chat";

    try {
      const response = await fetch(API_URL, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ message: userMessage.text }),
      });

      if (!response.ok) {
        throw new Error('Network response was not ok');
      }

      const data = await response.json();
      const botMessage = { text: data.response, sender: 'bot' };
      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = { text: "Sorry, I'm having trouble connecting to the server.", sender: 'bot' };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chat-widget-container">
      {isOpen && (
        <div className="chat-window">
          <div className="chat-header">
            <h3>
              <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" width="24px" height="24px" className="header-icon">
                <path d="M12 2a2 2 0 0 1 2 2c0 .74-.4 1.39-1 1.73V7h1a7 7 0 0 1 7 7h1a1 1 0 0 1 1 1v3a1 1 0 0 1-1 1h-1v1a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-1H2a1 1 0 0 1-1-1v-3a1 1 0 0 1 1-1h1a7 7 0 0 1 7-7V5.73C9.4 5.39 9 4.74 9 4a2 2 0 0 1 2-2M7.5 13a2.5 2.5 0 1 0 0 5 2.5 2.5 0 0 0 0-5m9 0a2.5 2.5 0 1 0 0 5 2.5 2.5 0 0 0 0-5"/>
              </svg>
              AI Assistant
            </h3>
            <div className="header-actions">
              {session && (
                <button className="logout-btn" onClick={handleLogout} title="Logout">
                  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" width="16px" height="16px">
                    <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4v2H5v14h4v2zm7-10v-4l7 5-7 5v-4H9v-2h7z"/>
                  </svg>
                </button>
              )}
              <button className="close-btn" onClick={toggleChat}>×</button>
            </div>
          </div>

          <div className="chat-messages">
            {messages.length === 0 && (
              <div className="chat-welcome">
                <p>
                  {session ? `Welcome back, ${session.user.name}!` : "Hello! How can I help you today?"}
                </p>
              </div>
            )}
            {messages.map((msg, index) => (
              <div key={index} className={`message ${msg.sender}`}>
                {msg.text}
              </div>
            ))}
            {isLoading && <div className="message bot">Thinking...</div>}
            <div ref={messagesEndRef} />
          </div>
          <form className="chat-input-form" onSubmit={handleSendMessage}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder={isRecording ? "Listening..." : "Ask about the book..."}
              disabled={isLoading}
            />
            <button 
              type="button" 
              className={`voice-btn ${isRecording ? 'recording' : ''}`}
              onClick={isRecording ? stopRecording : startRecording}
              disabled={isLoading}
              title="Voice Input"
            >
              {isRecording ? (
                <span className="recording-indicator"></span>
              ) : (
                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" width="20px" height="20px">
                  <path d="M12 14c1.66 0 3-1.34 3-3V5c0-1.66-1.34-3-3-3S9 3.34 9 5v6c0 1.66 1.34 3 3 3z"/>
                  <path d="M17 11c0 2.76-2.24 5-5 5s-5-2.24-5-5H5c0 3.53 2.61 6.43 6 6.92V21h2v-3.08c3.39-.49 6-3.39 6-6.92h-2z"/>
                </svg>
              )}
            </button>
            <button type="submit" disabled={isLoading}>Send</button>
          </form>
        </div>
      )}
      <button className="chat-toggle-btn" onClick={toggleChat}>
        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" width="24px" height="24px">
          <path d="M12 2c1.1 0 2 .9 2 2h4v2h-4v1h5v8h-2v4h-2v-4h-6v4H7v-4H5V7h5V5H6V4h6V2zM7 9v2h2V9H7zm8 0v2h2V9h-2z"/>
        </svg>
        {!isOpen && (
          <div className="chat-greeting-tooltip">
            <div className="greeting-icon">👋</div>
            <div className="greeting-text">
              <strong>Hello!</strong>
              <br />
              How can I assist you?
            </div>
          </div>
        )}
      </button>
    </div>
  );
};

export default ChatWidget;
