#ifndef ELLIPSE
#define ELLIPSE

#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <string>


namespace line_extraction
{

class Ellipse
{

private:
  // line_extraction::Line _line;
  double _r1;
  double _r2;
  double _p1;
  double _p2;
  double _rho;
  double _alpha;
  double _gamma;
  Eigen::Matrix2f _R;
  bool _in_collision;
  std::vector<double> _hyper_normal;

  std::array<double, 2>  _p_point;
  std::vector<double> _speed;



public:
  //Constructor/Destructor
  Ellipse(double posx,double posy, std::string type);
  Ellipse(double posx,double posy, double alpha, double r1=0.5, double r2 = 0.4);
  ~Ellipse();


  std::vector<double[]> calc_points();
  static double sBuffer;

  double  getAngle();
  double  getWidth();
  void    setWidth(double r2){_r2 = r2;};
  double  getHeight();
  void    setHeight(double r1){_r1 = r1;};
  double  getP1();
  double  getP2();
  double  getAlpha();
  double  getRho();
  Eigen::Matrix2f getR();
  void setR(Eigen::Matrix2f R){_R = R;};

  std::array<double, 2>&  getPPoint();
  std::vector<double> getSpeed(){return _speed;};
  void setSpeed(std::vector<double> speed){_speed = speed;};
  void setPPoint(double x, double y);
  void setGamma(double gamma);
  double getGamma();
  bool onLine(std::array<double, 2>& point);
  bool getInCollision() {return _in_collision;};
  void setInCollision(bool b) {_in_collision = b;};
  void setHyperNormal(std::vector<double> n) {_hyper_normal = n;};
  std::vector<double> getHyperNormal() {return _hyper_normal;};


};

}
#endif
