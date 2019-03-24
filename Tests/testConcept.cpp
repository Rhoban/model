#include <iostream>
#include <cassert>
#include "TimeSeries/TimeSeries.hpp"
#include "TimeSeries/Concept.hpp"

class TestConcept1 : public Leph::Concept
{
public:
  virtual std::string name() const override
  {
    return "TestConcept1";
  }
  virtual size_t inputSize() const override
  {
    return 2;
  }
  virtual size_t outputSize() const override
  {
    return 2;
  }
  virtual size_t parameterSize() const override
  {
    return 0;
  }
  virtual Leph::MetaParameter defaultParameter(size_t index) const override
  {
    (void)index;
    return Leph::MetaParameter();
  }

protected:
  virtual bool doCompute(double time) override
  {
    if (!Concept::getInput(0)->isTimeValid(time) || !Concept::getInput(1)->isTimeValid(time))
    {
      return false;
    }

    Concept::getOutput(0)->append(time, Concept::getInput(0)->get(time) * 2.0);
    Concept::getOutput(1)->append(time, Concept::getInput(0)->get(time) + Concept::getInput(1)->get(time));

    return true;
  }
};

class TestConcept2 : public Leph::Concept
{
public:
  virtual std::string name() const override
  {
    return "TestConcept2";
  }
  virtual size_t inputSize() const override
  {
    return 1;
  }
  virtual size_t outputSize() const override
  {
    return 1;
  }
  virtual size_t parameterSize() const override
  {
    return 0;
  }
  virtual Leph::MetaParameter defaultParameter(size_t index) const override
  {
    (void)index;
    return Leph::MetaParameter();
  }

protected:
  virtual bool doCompute(double time) override
  {
    if (Concept::getInput(0)->isTimeValid(time) && Concept::getInput(0)->isTimeValid(time - 1.0))
    {
      Concept::getOutput(0)->append(time, Concept::getInput(0)->get(time) + Concept::getInput(0)->get(time - 1.0));
      return true;
    }
    else
    {
      return false;
    }
  }
};

void testCompute()
{
  Leph::TimeSeries in("in");
  Leph::TimeSeries out("out");

  in.append(1.0, 1.0);
  in.append(2.0, 2.0);
  in.append(3.0, 3.0);
  in.append(4.0, 4.0);

  TestConcept2 concept2;
  concept2.resetParameters();
  concept2.setInput(0, &in);
  concept2.setOutput(0, &out);

  assert(concept2.inputSize() == 1);
  assert(concept2.outputSize() == 1);
  assert(concept2.name() == "TestConcept2");
  assert(in.size() == 4);
  assert(out.size() == 0);

  assert(concept2.computeAtTime(1.5) == false);
  assert(in.size() == 4);
  assert(out.size() == 0);
  assert(concept2.computeAtTime(2.0) == true);
  assert(in.size() == 4);
  assert(out.size() == 1);
  assert(out[0].time == 2.0);
  assert(out[0].value == 2.0 + 1.0);

  assert(concept2.computePropagate() == true);
  assert(in.size() == 4);
  assert(out.size() == 3);
  assert(out[0].time == 4.0);
  assert(out[0].value == 3.0 + 4.0);
  assert(out[1].time == 3.0);
  assert(out[1].value == 2.0 + 3.0);
  in.save(std::cout);
  out.save(std::cout);

  assert(concept2.computePropagate() == false);
}

int main()
{
  Leph::TimeSeries in1("in1");
  Leph::TimeSeries in2("in2");
  Leph::TimeSeries out1("out1");
  Leph::TimeSeries out2("out2");

  in1.append(1.0, 1.0);
  in1.append(2.0, 2.0);
  in1.append(3.0, 3.0);
  in1.append(4.0, 4.0);

  in2.append(1.2, 1.5);
  in2.append(2.2, 2.5);
  in2.append(3.2, 3.5);
  in2.append(4.2, 4.5);

  TestConcept1 concept1;
  concept1.resetParameters();
  concept1.setInput(0, &in1);
  concept1.setInput(1, &in2);
  concept1.setOutput(0, &out1);
  concept1.setOutput(1, &out2);

  assert(concept1.inputSize() == 2);
  assert(concept1.outputSize() == 2);
  assert(concept1.name() == "TestConcept1");
  assert(in1.size() == 4);
  assert(in2.size() == 4);
  assert(out1.size() == 0);
  assert(out2.size() == 0);

  assert(concept1.computeAtTime(1.5) == true);
  assert(in1.size() == 4);
  assert(in2.size() == 4);
  assert(out1.size() == 1);
  assert(out2.size() == 1);
  assert(out1[0].time == 1.5);
  assert(out2[0].time == 1.5);
  assert(out1[0].value == 3.0);
  assert(out2[0].value == 3.3);

  assert(concept1.computePropagate() == true);
  assert(out1.size() == 6);
  assert(out2.size() == 6);
  assert(out1[0].time == 4.0);
  assert(out1[1].time == 3.2);
  assert(out1[2].time == 3.0);
  assert(out1[3].time == 2.2);
  assert(out2[0].time == 4.0);
  assert(out2[1].time == 3.2);
  assert(out2[2].time == 3.0);
  assert(out2[3].time == 2.2);
  assert(out1[0].value == 8.0);
  assert(out1[1].value == 6.4);
  assert(out1[2].value == 6.0);
  assert(out1[3].value == 4.4);
  in1.save(std::cout);
  in2.save(std::cout);
  out1.save(std::cout);
  out2.save(std::cout);

  assert(concept1.computePropagate() == false);

  testCompute();

  return 0;
}
