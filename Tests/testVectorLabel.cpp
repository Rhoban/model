#include <iostream>
#include <fstream>
#include <cassert>
#include "Types/VectorLabel.hpp"

int main()
{
  Leph::VectorLabel vect1(4);
  std::cout << vect1 << std::endl;
  Leph::VectorLabel vect2({ "l1", "l2", "l4", "l3" });
  std::cout << vect2 << std::endl;
  Leph::VectorLabel vect3(Eigen::VectorXd(3));
  std::cout << vect3 << std::endl;
  Leph::VectorLabel vect4({ "l3", "l2", "l1" }, Eigen::VectorXd(3));
  std::cout << vect4 << std::endl;

  std::cout << Leph::VectorLabel::mergeUnion(vect1, vect2) << std::endl;
  std::cout << Leph::VectorLabel::mergeUnion(vect1, vect1) << std::endl;

  std::cout << "Vect2" << std::endl;
  std::cout << vect2 << std::endl;
  std::cout << "Vect4" << std::endl;
  std::cout << vect4 << std::endl;
  std::cout << "Inter 2-4" << std::endl;
  std::cout << Leph::VectorLabel::mergeInter(vect2, vect4) << std::endl;
  std::cout << "Inter 4-4" << std::endl;
  std::cout << Leph::VectorLabel::mergeInter(vect4, vect4) << std::endl;

  Leph::VectorLabel vect6("test label 1", 1.0, "test label 2", 2.0);
  assert(vect6.exist("test label 1"));
  assert(vect6("test label 1") == 1.0);
  assert(!vect6.exist("test label 3"));
  assert(!vect6.exist("test label 4"));
  vect6.append("test label 3", 3.0, "test label 4", 4.0);
  assert(vect6.exist("test label 3"));
  assert(vect6.exist("test label 4"));
  std::cout << vect6 << std::endl;

  assert(vect1.vect().size() == 4);
  assert(vect1.size() == 4);
  vect1.vect()(0) = 1.0;
  assert(vect1.vect()(0) == 1.0);
  std::cout << vect1 << std::endl;

  assert(vect2.labels().size() == 4);
  assert(vect2.getLabel(0) == "l1");
  assert(vect2.getIndex("l1") == 0);

  vect2("l1") = 0.0;
  assert(vect2("l1") == 0.0);
  vect2("l1") = 1.0;
  assert(vect2("l1") == 1.0);
  std::cout << vect2 << std::endl;

  vect2(1) = 0.0;
  assert(vect2(1) == 0.0);
  vect2(1) = 2.0;
  assert(vect2(1) == 2.0);
  std::cout << vect2 << std::endl;

  vect2.writeToCSV();
  std::ofstream file("/tmp/testVectorLabel.csv");
  vect2.writeToCSV(file);

  vect2("l1") = 666;
  std::string csvStr = "#'t1' 't2' 'l3'\n0.1 0.2 42.0\n\n";
  vect2.readFromCSV(csvStr);
  vect2.writeToCSV(file);
  std::cout << vect2 << std::endl;
  file.close();

  std::ifstream fileIn("/tmp/testVectorLabel.csv");
  while (vect2.readFromCSV(fileIn))
  {
    std::cout << vect2 << std::endl;
  }
  fileIn.close();

  std::cout << Leph::VectorLabel::toSection("section:name") << std::endl;
  std::cout << Leph::VectorLabel::toSection("sectionname") << std::endl;
  std::cout << Leph::VectorLabel::toName("section:name") << std::endl;
  std::cout << Leph::VectorLabel::toName("sectionname") << std::endl;
  std::cout << std::endl;

  Leph::VectorLabel vect7("a:x", 1.0, "b:x", 2.0, "b:y", 3.0);
  Leph::VectorLabel vect8("a:x", 10.0, "b:y", 20.0);
  vect7.addOp(vect8, "b");
  std::cout << vect7 << std::endl;
  vect7.mulOp(0.5);
  std::cout << vect7 << std::endl;
  vect7.subOp(vect8, "a");
  std::cout << vect7 << std::endl;
  vect7.addOp(vect7, "b", "a");
  std::cout << vect7 << std::endl;
  vect7.addOp(100.0, "b");
  vect7.subOp(100.0, "b");
  vect7.mulOp(100.0, "b");
  vect7.divOp(100.0, "b");
  vect7.squareOp("b");
  vect7.sqrtOp("b");
  std::cout << vect7 << std::endl;

  Leph::VectorLabel vect9("a:x", 1.0, "a:y", 2.0, "b:x", 3.0);
  Leph::VectorLabel vect10("a:x", 1.0, "a:y", 4.0, "b:x", 6.0, "b:y", 5.0);
  std::cout << "Vect9" << std::endl;
  std::cout << vect9 << std::endl;
  std::cout << "Vect10" << std::endl;
  std::cout << vect10 << std::endl;
  vect9.mergeInter(vect10, "b");
  std::cout << "Vect9 union 10" << std::endl;
  std::cout << vect9 << std::endl;
  vect9.mergeUnion(vect10);
  std::cout << "Vect9 inter 10" << std::endl;
  std::cout << vect9 << std::endl;

  vect10("a:x")++;
  assert(Leph::VectorLabel::isEqualInter(vect9, vect10) == 0);
  assert(Leph::VectorLabel::isEqualInter(vect9, vect10, "b") == 1);

  assert(vect9.sum() == 16.0);
  assert(vect9.sum("a") == 5.0);
  assert(vect9.mean() == 4.0);
  std::cout << vect9.rename("a", "c") << std::endl;
  vect9.setOrAppend("a:z", 42.0);
  std::cout << vect9 << std::endl;
  vect9.setOrAppend("a:z", 69.0);
  std::cout << vect9 << std::endl;
  vect9.assignOp(vect9.rename("a", "c"), "c", "b");
  std::cout << vect9 << std::endl;

  return 0;
}
