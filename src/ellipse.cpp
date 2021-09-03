#include "laser_line_extraction/ellipse.h"


namespace line_extraction
{

double Ellipse::sBuffer = 0.50;

/*/ Ellipse build to represent the IR of the Robot
Ellipse::Ellipse(double posx,double posy, std::string type):
  _p1(1.0),
  _p2(1.0)
{ 
  _p_point = {{posx, posy}};
  _type = type;
  _in_collision = false;
  _R << 1.0, 0.0, 0.0, 1.0;
  std::vector<double> n;
  n.push_back(0.0);
  n.push_back(1.0);
  setHyperNormal(n);
  _r2 = 0.6;
  _r1 = 0.5;
  _rho = 1.88;

  _speed.push_back(0.0);
  _speed.push_back(0.0);
}*/


// Ellipse build from position and alpha
Ellipse::Ellipse(double posx,double posy, double alpha, double r1, double r2):
  _p1(4.0),
  _p2(4.0),
  _rho(0.03)
{
  _p_point = {{posx, posy}};
  _alpha = alpha;
  _r1 = r1;
  _r2 = r2;
  _in_collision = false;
  double cosangle = cos(alpha);
  double sinangle = sin(alpha);
  _R << cosangle, sinangle, -sinangle, cosangle;
  _speed.push_back(0.0);
  _speed.push_back(0.0);
  std::vector<double> n;
  n.push_back(0.0);
  n.push_back(1.0);
  setHyperNormal(n);
}



Ellipse::~Ellipse()
{

}


double Ellipse::getWidth() {
  return _r2;
}

double Ellipse::getHeight() {
  return _r1;
}

double Ellipse::getP1() {
  return _p1;
}

double Ellipse::getP2() {
  return _p2;
}

double Ellipse::getAlpha() {
  return _alpha; //atan2(-_R(1,0),_R(0,0));
}

double Ellipse::getRho() {
  return _rho;
}

Eigen::Matrix2f Ellipse::getR() {
  return _R;
}


std::array<double, 2>& Ellipse::getPPoint() {
  return _p_point;
}

void Ellipse::setPPoint(double x, double y) {
  _p_point = {{x, y}};
}

void Ellipse::setGamma(double gamma) {
  _gamma = gamma;
}

double Ellipse::getGamma() {
  return _gamma;
}


}

