import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { authClient } from '../lib/auth-client';
import styles from './auth.module.css';

export default function AuthPage() {
  const [authMode, setAuthMode] = useState('signin');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');

  const handleAuth = async (e) => {
    e.preventDefault();
    setError('');
    setSuccess('');
    setIsLoading(true);

    console.log("Attempting authentication...", { authMode, email });

    try {
      if (authMode === 'signup') {
        const { data, error } = await authClient.signUp.email({
          email,
          password,
          name,
        });
        if (error) {
            console.error("Sign up error:", error);
            throw error;
        }
        console.log("Sign up success:", data);
        setSuccess('Account created successfully! You can now sign in.');
        setAuthMode('signin');
      } else {
        const { data, error } = await authClient.signIn.email({
          email,
          password,
        });
        if (error) {
            console.error("Sign in error:", error);
            throw error;
        }
        console.log("Sign in success:", data);
        window.location.href = '/'; // Redirect to home on success
      }
    } catch (err) {
      console.error("Auth exception:", err);
      setError(err.message || err.statusText || 'Authentication failed. Please check your credentials and connection.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout title="Authentication" description="Sign In or Sign Up">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1>{authMode === 'signin' ? 'Welcome Back' : 'Create Account'}</h1>
          
          <form onSubmit={handleAuth} className={styles.authForm}>
            {authMode === 'signup' && (
              <div className={styles.inputGroup}>
                <label>Name</label>
                <input
                  type="text"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  required
                  placeholder="John Doe"
                />
              </div>
            )}
            
            <div className={styles.inputGroup}>
              <label>Email</label>
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
                placeholder="you@example.com"
              />
            </div>

            <div className={styles.inputGroup}>
              <label>Password</label>
              <input
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                placeholder="••••••••"
              />
            </div>

            {error && <div className={styles.errorMessage}>{error}</div>}
            {success && <div className={styles.successMessage}>{success}</div>}

            <button type="submit" disabled={isLoading} className={styles.submitBtn}>
              {isLoading ? 'Processing...' : (authMode === 'signin' ? 'Sign In' : 'Sign Up')}
            </button>
          </form>

          <div className={styles.authSwitch}>
            {authMode === 'signin' ? "Don't have an account? " : "Already have an account? "}
            <button 
              onClick={() => {
                setAuthMode(authMode === 'signin' ? 'signup' : 'signin');
                setError('');
                setSuccess('');
              }}
              className={styles.switchBtn}
            >
              {authMode === 'signin' ? 'Sign Up' : 'Sign In'}
            </button>
          </div>
        </div>
      </div>
    </Layout>
  );
}
