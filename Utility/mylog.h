#ifndef __MYLOG_H__ 
#define __MYLOG_H__ 


#ifdef __cplusplus 
extern "C" { 
#endif 

#include <stdio.h> 


/* 日志级别 */ 
#define ELOG_LVL_ASSERT 0 
#define ELOG_LVL_ERROR 1 
#define ELOG_LVL_WARN 2 
#define ELOG_LVL_INFO 3 
#define ELOG_LVL_DEBUG 4 
#define ELOG_LVL_VERBOSE 5 


/* 设置日志级别 */ 
#define ELOG_OUTPUT_LVL ELOG_LVL_VERBOSE 


/* 断言(Assert) */ 
#define LOG_A(args,...) do{ if (ELOG_OUTPUT_LVL >= ELOG_LVL_ASSERT) { printf("[A/%s Line:%.4d] " args "\r\n", __FILE__, __LINE__, ##__VA_ARGS__); } }while(0) 
	
/* 错误(Error) */ 
#define LOG_E(args,...) do{ if (ELOG_OUTPUT_LVL >= ELOG_LVL_ASSERT) { printf("[E/%s Line:%.4d] " args "\r\n", __FILE__, __LINE__, ##__VA_ARGS__); } }while(0) 
	
/* 警告(Warn) */ 
#define LOG_W(args,...) do{ if (ELOG_OUTPUT_LVL >= ELOG_LVL_WARN) { printf("[W/%s Line:%.4d] " args "\r\n", __FILE__, __LINE__, ##__VA_ARGS__); } }while(0) 
	
/* 信息(Info) */ 
#define LOG_I(args,...) do{ if (ELOG_OUTPUT_LVL >= ELOG_LVL_INFO) { printf("[I/%s Line:%.4d] " args "\r\n", __FILE__, __LINE__, ##__VA_ARGS__); } }while(0)   
	
/* 调试(Debug) */ 
#define LOG_D(args,...) do{ if (ELOG_OUTPUT_LVL >= ELOG_LVL_DEBUG) { printf("[D/%s Line:%.4d] " args "\r\n", __FILE__, __LINE__, ##__VA_ARGS__); } }while(0)   
	
/* 详细(Verbose) */ 
#define LOG_V(args,...) do{ if (ELOG_OUTPUT_LVL >= ELOG_LVL_VERBOSE) { printf("[V/%s Line:%.4d] " args "\r\n", __FILE__, __LINE__, ##__VA_ARGS__); } }while(0) 


#ifdef __cplusplus 
} 
#endif 

#endif /* __MYLOG_H__ */