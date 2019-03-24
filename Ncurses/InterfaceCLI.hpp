#ifndef LEPH_INTERFACECLI_HPP
#define LEPH_INTERFACECLI_HPP

#include <vector>
#include <iostream>
#include <map>
#include <functional>
#include <string>
#include <ncurses.h>
#include <Types/VectorLabel.hpp>

namespace Leph
{
/**
 * InterfaceCLI
 *
 * Generic CLI interface based on
 * NCurses and VectorLabel
 */
class InterfaceCLI
{
public:
  /**
   * Initialization
   */
  InterfaceCLI(const std::string& title = "Ncurses Interface");

  /**
   * Desalocation (exit Ncurses)
   */
  ~InterfaceCLI();

  /**
   * Explicitly exit NCurses interface end
   * go back the terminal
   */
  void quit();

  /**
   * Add a VectorLabel under given section name
   * for write parameters as reference
   * Vector pointer are used. Do no destruct
   * the given vector before the end of InterfaceCLI
   * instance life
   * (Optional section filter can be given)
   */
  void addParameters(const std::string& sectionName, VectorLabel& vector, const std::string& filter = "");

  /**
   * Add a VectorLabel under given section name
   * for values real time display as reference
   * Vector pointer are used. Do no destruct
   * the given vector before the end of InterfaceCLI
   * instance life
   * (Optional section filter can be given)
   */
  void addMonitors(const std::string& sectionName, VectorLabel& vector, const std::string& filter = "");

  /**
   * Add an user defined input binding with
   * input key (NCurses), a helping message and
   * a callback function
   */
  void addBinding(int key, const std::string& helpMsg, std::function<void()> handler);

  /**
   * Add a user defined status string line
   * as string reference
   * String pointer are used. Do no destruct
   * the given string before the end of InterfaceCLI
   * instance life
   */
  void addStatus(std::string& line);

  /**
   * Return the internal ostringstream
   * for appending message to terminal output.
   */
  std::ostream& terminalOut();

  /**
   * Refresh screen if needed and handle keybord input.
   * Return false if exit is asked. Else return true.
   * Monitors and Status values are not updated if updateAll is
   * false (save up terminal update time).
   */
  bool tick(bool updateAll = true);

  /**
   * Draw Ncurses title windows
   * (Force window refresh)
   */
  void drawTitleWin();

  /**
   * Draw Ncurses status windows
   * (Force window refresh)
   */
  void drawStatusWin();

  /**
   * Draw Ncurses terminal message windows
   * (Force window refresh)
   */
  void drawTerminalWin();

  /**
   * Draw Ncurses parameters windows
   * (Force window refresh)
   */
  void drawParamsWin();

  /**
   * Draw Ncurses monitors windows
   * (Force window refresh)
   */
  void drawMonitorsWin();

private:
  /**
   * Structure for input event handler
   */
  struct InputBinding
  {
    int key;
    std::function<void()> handler;
    std::string help;
  };

  /**
   * Typedef for parameters and monitor
   * First is VectorLabel
   * Second is optional filter
   */
  typedef std::pair<VectorLabel*, std::string> FilteredVectorLabel;

  /**
   * Ncurses windows pointer to
   * title, status, parameters and
   * monitors windows
   */
  WINDOW* _titleWin;
  WINDOW* _statusWin;
  WINDOW* _terminalWin;
  WINDOW* _paramsWin;
  WINDOW* _monitorsWin;

  /**
   * Interface title
   */
  std::string _title;

  /**
   * VectorLabel pointer container
   * for parameters update
   * associated with section name
   * And sum of parameter
   */
  std::map<std::string, FilteredVectorLabel> _parameters;
  size_t _sumParams;

  /**
   * VectorLabel pointer container
   * for values monitors
   * associated with section name
   * And sum of monitor
   */
  std::map<std::string, FilteredVectorLabel> _monitors;
  size_t _sumMonitors;

  /**
   * Summed index of selected parameter
   */
  size_t _selected;

  /**
   * Parameters incr/decr delta
   */
  double _paramsDelta;

  /**
   * User added input handler
   */
  std::vector<InputBinding> _userBindings;

  /**
   * User defined status
   */
  std::vector<std::string*> _userStatus;

  /**
   * Terminal rolling message output
   * string stream
   */
  bool _isTerminalUpdated;
  std::ostringstream _terminalStream;

  /**
   * Return a reference to selected
   * parameters value
   */
  double& getSelected();
};

}  // namespace Leph

#endif
