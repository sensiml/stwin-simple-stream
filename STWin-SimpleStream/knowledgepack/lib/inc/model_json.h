#ifndef __MODEL_JSON_H__
#define __MODEL_JSON_H__

const char recognition_model_string_json[] = {"{"\NumModels"\:1,"\ModelIndexes"\:{"\0"\:"\tf_all_cnn_1"\},"\ModelDescriptions"\:[{"\Name"\:"\tf_all_cnn_1"\,"\ClassMaps"\:{"\1"\:"\Abnormal"\,"\2"\:"\Off"\,"\3"\:"\On"\,"\0"\:"\Unknown"\},"\ModelType"\:"\TFMicro"\}]}"};


int recognition_model_string_json_len = sizeof(recognition_model_string_json);

#endif /* __MODEL_JSON_H__ */