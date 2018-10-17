#ifndef NCLT_PLUGIN_H_
#define NCLT_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base.h>
#include <console-common/console.h>

namespace nclt {

class NcltPlugin : public common::ConsolePluginBase {
 public:
  explicit NcltPlugin(common::Console* console);

  std::string getPluginId() const override {
    return "nclt";
  }

 private:
  int importGroundTruthDataFromCsv() const;
};

}  // namespace nclt

#endif  // NCLT_PLUGIN_H_
