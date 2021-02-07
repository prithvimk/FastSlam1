#include <iostream>
#include <vector>
#include <cmath>

#include <eigen3/Eigen/Dense>



double
DT = 0.1,
SIM_TIME = 50.0,
MAX_RANGE = 20.0,
M_DIST_TH = 2.0,
STATE_SIZE = 3,
LM_SIZE = 2,
N_PARTICLE = 100,
NTH = N_PARTICLE / 1.5;

Eigen::MatrixXd calc_input(double time);

class Particle {
  public:
    double w, x, y, yaw;
    Eigen::MatrixXd lm, lmP;
    Particle(int n_landmark) {
      w = 1.0 / N_PARTICLE;
      x = 0.0;
      y = 0.0;
      yaw = 0.0;
      // landmark x-y positions
      lm = Eigen::MatrixXd::Zero(n_landmark, LM_SIZE);
      lmP = Eigen::MatrixXd::Zero(n_landmark * LM_SIZE, LM_SIZE);
    }
    Particle normalize_weight();

  std::vector<Particle> fast_slam1(std::vector<Particle> particles, Eigen::MatrixXd u, Eigen::MatrixXd z) {
    particles = predict_particles(particles, u);
    
    particles = update_with_observation(particles, z);

    particles = resampling(particles);

    return particles;
  }

  std::vector<Particle> normalize_weight(std::vector<Particle> particles) {
    double sum_w = 0.0;
    for (int i = 0; i < particles.size(); i++) {
      sum_w += particles[i].w;
    }

    try
    {
      for (int i = 0; i < N_PARTICLE; i++) {
        particles[i].w /= sum_w;
      }
    }
    catch(const std::exception& e)
    {
      for (int i = 0; i < N_PARTICLE; i++) {
        particles[i].w = 1.0 / N_PARTICLE;
      }

      return particles;
    }
    
    return particles;
  }

  std::vector<Particle> predict_particles(std::vector<Particle> particles, Eigen::MatrixXd u) {
    for (int i = 0; i < N_PARTICLE; i++) {
      Eigen::MatrixXd px = Eigen::MatrixXd::Zero(STATE_SIZE, 1);
      px(0, 0) = particles[i].x;
      px(1, 0) = particles[i].y;
      px(2, 0) = particles[i].yaw;
      Eigen::MatrixXd ud = u + Eigen::MatrixXd::Random(1, 2);
    }
  }

  Particle add_new_landmark(Particle particle, Eigen::MatrixXd z, Eigen::MatrixXd Q_cov) {
    double r = z(0);
    double b = z(1);
    int lm_id = int(z(2));

    double s = sin(pi_2_pi(particle.yaw + b));
    double c = cos(pi_2_pi(particle.yaw + b));

    particle.lm[lm_id, 0] = particle.x + r * c;
    particle.lm[lm_id, 1] = particle.y + r * s;

    //covariance
    double dx = r * c;
    double dy = r * s;
    double d2 = dx * dx + dy * dy;
    double d = sqrt(d2);
    Eigen::Matrix2d Gz;
    Gz << dx / d, dy / d,
          -dy / d2, dx / d2;
    //particle.lmP.block()

    return particle;
  }

  Eigen::MatrixXd calc_final_state(std::vector<Particle> particles) {
    Eigen::MatrixXd xEst = Eigen::MatrixXd::Zero(STATE_SIZE, 1);

    particles = normalize_weight(particles);

    for (int i = 0; i < N_PARTICLE; i++) {
      xEst(0, 0) += particles[i].w * particles[i].x;
      xEst(1, 0) += particles[i].w * particles[i].y;
      xEst(2, 0) += particles[i].w * particles[i].yaw;
    }

    xEst(2, 0) = pi_2_pi(xEst(2, 0));

    return xEst;
  }

  std::vector<Eigen::MatrixXd> compute_jacobians(Particle particle, Eigen::MatrixXd xf, Eigen::MatrixXd Pf, Eigen::MatrixXd Q_cov) {
    double dx = xf(0, 0) - particle.x;
    double dy = xf(1, 0) - particle.y;
    double d2 = dx * dx + dy * dy;
    double d = sqrt(d2);

    Eigen::MatrixXd zp(2, 1), Hv(3, 3), Hf(2, 2), Sf;

    zp << d, 
          pi_2_pi(atan2(dy, dx) - particle.yaw);

    Hv << -dx / d, -dy / d, 0.0,
           dy / d2, -dx / d2, -1.0;

    Hf << dx / d, dy / d,
          -dy / d2, dx / d2;
    
    Sf = Hf * Pf * Hf.transpose() + Q_cov;

    std::vector<Eigen::MatrixXd> jacob;
    jacob.insert(jacob.end(), {zp, Hv, Hf, Sf});
    
    return jacob;
  }

  //incomplete
  std::vector<Eigen::MatrixXd> update_kf_cholesky(Eigen::MatrixXd xf, Eigen::MatrixXd Pf, Eigen::MatrixXd v, Eigen::MatrixXd Q_cov, Eigen::MatrixXd Hf) {
    Eigen::MatrixXd PHt = Pf * Hf.transpose();
    Eigen::MatrixXd S = Hf * PHt + Q_cov;

    S = (S + S.transpose()) * 0.5;
    Eigen::MatrixXd s_chol;

    return;
  }
  //incomplete

  Particle update_landmark(Particle particle, Eigen::MatrixXd z, Eigen::MatrixXd Q_cov) {
    
  }

};

Eigen::MatrixXd calc_input(double time) {
  double v, yaw_rate;
  if (time <= 3.0) {
    v = 0.0;
    yaw_rate = 0.0;
  }
  else {
    v = 1.0;
    yaw_rate = 0.1;
  }

  Eigen::MatrixXd u(v, yaw_rate);

  return u;
}

double pi_2_pi(double angle) {
  return fmod((angle + M_PI), (2 * M_PI) - M_PI);
}

Eigen::MatrixXd motion_model(Eigen::MatrixXd x, Eigen::MatrixXd u) {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(3, 3);
  Eigen::MatrixXd B(3, 2);
  B <<  DT * cos(x(2, 0)), 0,
        DT * sin(x(2, 0)), 0,
        0.0, DT;
  
  x = F * x + B * u;

  x(2, 0) = pi_2_pi(x(2, 0));

  return x;
}

int main() {
  Eigen::MatrixXd Q, R;
  Q << 3.0, 0.0,
       0.0, (10.0 * M_PI / 180);
  R << 1.0, 0.0,
       0.0, (20.0 * M_PI / 180);
  Q = Q * Q;
  R = R * R;
  return 0;
}