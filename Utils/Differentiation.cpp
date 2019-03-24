#include <stdexcept>
#include "Utils/Differentiation.hpp"
#include "Spline/PolyFit.hpp"

namespace Leph
{
Differentiation::Differentiation(unsigned int size, unsigned int degree) : _size(size), _degree(degree)
{
  if (_size < _degree + 1)
  {
    throw std::logic_error("Differentiation not enough size for fitting");
  }
}

void Differentiation::add(double time, const VectorLabel& vect)
{
  if (_container.size() != 0 && _container.front().second.size() != vect.size())
  {
    throw std::logic_error("Differentiation invalid vector size");
  }
  if (_container.size() != 0 && _container.back().first > time)
  {
    throw std::logic_error("Differentiation not increasing time");
  }

  _container.push_back({ time, vect });

  while (_container.size() > _size)
  {
    _container.pop_front();
  }
}

bool Differentiation::isFull() const
{
  return (_container.size() >= _size);
}

double Differentiation::minTime() const
{
  if (_container.size() == 0)
  {
    throw std::logic_error("Differentiation empty");
  }
  return _container.front().first;
}
double Differentiation::maxTime() const
{
  if (_container.size() == 0)
  {
    throw std::logic_error("Differentiation empty");
  }
  return _container.back().first;
}

VectorLabel Differentiation::position(double t)
{
  if (_container.size() == 0)
  {
    throw std::logic_error("Differentiation empty");
  }

  VectorLabel pos = _container.front().second;
  for (size_t i = 0; i < pos.size(); i++)
  {
    PolyFit fit(_degree);
    for (const auto& p : _container)
    {
      fit.add(p.first, p.second(i));
    }
    Polynom polynom = fit.fitting();
    pos(i) = polynom.pos(t);
  }

  return pos;
}
VectorLabel Differentiation::velocity(double t)
{
  if (_container.size() == 0)
  {
    throw std::logic_error("Differentiation empty");
  }

  VectorLabel vel = _container.front().second;
  for (size_t i = 0; i < vel.size(); i++)
  {
    PolyFit fit(_degree);
    for (const auto& p : _container)
    {
      fit.add(p.first, p.second(i));
    }
    Polynom polynom = fit.fitting();
    vel(i) = polynom.vel(t);
  }

  return vel;
}
VectorLabel Differentiation::acceleration(double t)
{
  if (_container.size() == 0)
  {
    throw std::logic_error("Differentiation empty");
  }

  VectorLabel acc = _container.front().second;
  for (size_t i = 0; i < acc.size(); i++)
  {
    PolyFit fit(_degree);
    for (const auto& p : _container)
    {
      fit.add(p.first, p.second(i));
    }
    Polynom polynom = fit.fitting();
    acc(i) = polynom.acc(t);
  }

  return acc;
}

}  // namespace Leph
