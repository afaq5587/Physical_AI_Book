import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className={styles.heroBackground}>
        <div className={styles.floatingShape1}></div>
        <div className={styles.floatingShape2}></div>
        <div className={styles.floatingShape3}></div>
      </div>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.badge}>
            <span className={styles.badgeIcon}>🤖</span>
            <span className={styles.badgeText}>13-Week Comprehensive Course</span>
          </div>
          <Heading as="h1" className={styles.heroTitle}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
          <p className={styles.heroDescription}>
            Master the future of robotics with hands-on experience in ROS 2, 
            Digital Twins, AI-powered control systems, and Vision-Language-Action models.
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--lg', styles.primaryButton)}
              to="/intro">
              <span className={styles.buttonIcon}>🚀</span>
              Start Learning
            </Link>
            <Link
              className={clsx('button button--lg', styles.secondaryButton)}
              to="/intro#course-overview">
              <span className={styles.buttonIcon}>📚</span>
              View Curriculum
            </Link>
          </div>
          <div className={styles.stats}>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>4</div>
              <div className={styles.statLabel}>Modules</div>
            </div>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>13</div>
              <div className={styles.statLabel}>Weeks</div>
            </div>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>∞</div>
              <div className={styles.statLabel}>Possibilities</div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function FeatureSection() {
  const features = [
    {
      title: 'ROS 2 Mastery',
      icon: '⚙️',
      description: 'Build robust robotic systems with the industry-standard Robot Operating System 2.',
      gradient: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)'
    },
    {
      title: 'Digital Twin Simulation',
      icon: '🔮',
      description: 'Create realistic virtual environments with Gazebo and Unity for safe testing.',
      gradient: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)'
    },
    {
      title: 'AI-Powered Control',
      icon: '🧠',
      description: 'Leverage NVIDIA Isaac for photorealistic simulation and reinforcement learning.',
      gradient: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)'
    },
    {
      title: 'Vision-Language-Action',
      icon: '👁️',
      description: 'Integrate cutting-edge VLA models for advanced human-robot interaction.',
      gradient: 'linear-gradient(135deg, #43e97b 0%, #38f9d7 100%)'
    }
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className={styles.sectionTitle}>What You'll Learn</h2>
        <div className={styles.featureGrid}>
          {features.map((feature, idx) => (
            <div key={idx} className={styles.featureCard}>
              <div className={styles.featureIcon} style={{background: feature.gradient}}>
                {feature.icon}
              </div>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDescription}>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Master Physical AI and Humanoid Robotics with our comprehensive 13-week course">
      <HomepageHeader />
      <main>
        <FeatureSection />
      </main>
    </Layout>
  );
}
