#ifndef INCLUDE_NCLT_TOOLS_CSV_PARSING_H_
#define INCLUDE_NCLT_TOOLS_CSV_PARSING_H_

#include <string>
#include <vector>

#include <aslam/common/pose-types.h>

namespace nclt {

void getTimeAndTransformationFromDataRecord(
    const std::vector<std::string>& data_record, int64_t* timestamp_nanoseconds,
    aslam::Transformation* T_fi_fo);

}  // namespace nclt

#endif  // INCLUDE_NCLT_TOOLS_CSV_PARSING_H_
