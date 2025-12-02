import React, { useEffect, useState } from 'react';

const TranslateButton = () => {
  const [isTranslated, setIsTranslated] = useState(false);

  useEffect(() => {
    // Check cookie on load to set state
    const cookies = document.cookie.split(';');
    const hasTranslationCookie = cookies.some(c => c.trim().startsWith('googtrans='));
    setIsTranslated(hasTranslationCookie);

    // Initialize Google Translate Script
    if (!document.getElementById('google-translate-script')) {
      window.googleTranslateElementInit = () => {
        new window.google.translate.TranslateElement(
          {
            pageLanguage: 'en',
            includedLanguages: 'ur,en',
            layout: window.google.translate.TranslateElement.InlineLayout.SIMPLE,
            autoDisplay: false,
          },
          'google_translate_element'
        );
      };

      const script = document.createElement('script');
      script.id = 'google-translate-script';
      script.src = 'https://translate.google.com/translate_a/element.js?cb=googleTranslateElementInit';
      script.async = true;
      document.body.appendChild(script);
    }
  }, []);

  const handleTranslate = () => {
    // Set cookie to translate from English to Urdu
    document.cookie = "googtrans=/en/ur; path=/; domain=" + window.location.hostname;
    document.cookie = "googtrans=/en/ur; path=/"; // Fallback for localhost
    window.location.reload();
  };

  const handleReset = () => {
    // Clear cookies
    document.cookie = "googtrans=; path=/; domain=" + window.location.hostname + "; expires=Thu, 01 Jan 1970 00:00:00 UTC";
    document.cookie = "googtrans=; path=/; expires=Thu, 01 Jan 1970 00:00:00 UTC";
    window.location.reload();
  };

  return (
    <div className="translate-wrapper" style={{ marginBottom: '20px' }}>
      <button
        onClick={isTranslated ? handleReset : handleTranslate}
        style={{
          background: 'linear-gradient(135deg, #2563eb 0%, #1d4ed8 100%)',
          color: 'white',
          border: 'none',
          padding: '10px 20px',
          borderRadius: '8px',
          cursor: 'pointer',
          fontWeight: '600',
          fontSize: '14px',
          display: 'inline-flex',
          alignItems: 'center',
          gap: '8px',
          boxShadow: '0 4px 6px -1px rgba(37, 99, 235, 0.2)',
          transition: 'all 0.2s ease'
        }}
        onMouseOver={(e) => e.currentTarget.style.transform = 'translateY(-1px)'}
        onMouseOut={(e) => e.currentTarget.style.transform = 'translateY(0)'}
      >
        <span style={{ fontSize: '18px' }}>🌐</span>
        {isTranslated ? 'Show Original (English)' : 'Translate to Urdu'}
      </button>

      {/* Visually hidden Google Translate Widget */}
      <div id="google_translate_element" style={{ position: 'absolute', width: '1px', height: '1px', overflow: 'hidden', clip: 'rect(0,0,0,0)', whiteSpace: 'nowrap' }}></div>
      
      <style>{`
        /* CRITICAL FIXES: Prevent Google Translate from breaking the layout */
        body {
            top: 0 !important;
            position: static !important;
        }
        .goog-te-banner-frame {
            display: none !important;
            visibility: hidden !important;
            height: 0 !important;
            width: 0 !important;
            z-index: -1000 !important;
        }
        body.skiptranslate {
            margin-top: 0 !important;
        }
        #goog-gt-tt {
            display: none !important;
            visibility: hidden !important;
        }
      `}</style>
    </div>
  );
};

export default TranslateButton;
