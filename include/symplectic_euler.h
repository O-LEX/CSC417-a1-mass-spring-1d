//Input:
//  q - generalized coordiantes for the mass-spring system
//  qdot - generalized velocity for the mass spring system
//  dt - the time step in seconds
//  mass - the mass
//  force(q, qdot) - a function that computes the force acting on the mass as a function. This takes q and qdot as parameters.
//Output:
//  q - set q to the updated generalized coordinate using Symplectic Euler time integration
//  qdot - set qdot to the updated generalized velocity using Symplectic Euler time integration

template<typename FORCE> 
inline void symplectic_euler(Eigen::VectorXd &q, Eigen::VectorXd &qdot, double dt, double mass,  FORCE &force) {
    Eigen::VectorXd f(q.size());
    force(f, q, qdot);

    // 加速度を計算
    Eigen::VectorXd acceleration = f / mass;

    // 速度と位置を更新
    qdot += acceleration * dt;
    q += qdot * dt;
}