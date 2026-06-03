#pragma once
#include "woan_api/woan_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file woan_version.h
 * @brief WoanArm API版本信息定义
 * 
 * 本文件定义了WoanArm API的版本号，采用语义化版本控制（Semantic Versioning）规范：
 * 版本格式：MAJOR.MINOR.PATCH
 * 
 * @section version_semantics 版本号语义说明
 * 
 * - **MAJOR（主版本号）**: 
 *   当发生不兼容的API变更时递增。例如：
 *   - 删除或重命名公共API函数
 *   - 修改函数签名（参数类型、数量、返回值类型）
 *   - 改变数据结构布局
 *   - 移除已弃用的功能
 *   主版本号变更意味着可能需要修改调用代码才能适配新版本。
 * 
 * - **MINOR（次版本号）**: 
 *   当以向后兼容的方式添加新功能时递增。例如：
 *   - 添加新的API函数（不影响现有函数）
 *   - 扩展现有数据结构（向后兼容）
 *   - 添加新的可选参数（使用默认值）
 *   - 性能优化（不改变行为）
 *   次版本号变更意味着新版本完全兼容旧版本的API。
 * 
 * - **PATCH（补丁号）**: 
 *   当进行向后兼容的问题修复时递增。例如：
 *   - 修复bug
 *   - 修复内存泄漏
 *   - 修复崩溃问题
 *   - 修复逻辑错误
 *   补丁号变更意味着只修复问题，不添加新功能，也不改变API。
 * 
 * @section version_examples 版本号示例
 * 
 * - 1.0.0 -> 2.0.0: 主版本号变更，API不兼容
 * - 1.0.0 -> 1.1.0: 次版本号变更，添加新功能但保持兼容
 * - 1.0.0 -> 1.0.1: 补丁号变更，仅修复bug
 * 
 * @section version_checking 版本检查建议
 * 
 * 在调用API前，建议检查版本号以确保兼容性：
 * - 如果主版本号不同，需要检查API变更并更新代码
 * - 如果次版本号不同，可以安全使用，但建议查看新功能
 * - 如果补丁号不同，可以安全使用，通常只是bug修复
 */

/**
 * @def WOAN_API_VERSION_MAJOR
 * @brief 主版本号
 * 
 * 当前值：1
 * 含义：API的主版本号，当发生不兼容的API变更时递增
 */
#define WOAN_API_VERSION_MAJOR 1

/**
 * @def WOAN_API_VERSION_MINOR
 * @brief 次版本号
 * 
 * 当前值：0
 * 含义：API的次版本号，当以向后兼容的方式添加新功能时递增
 */
#define WOAN_API_VERSION_MINOR 0

/**
 * @def WOAN_API_VERSION_PATCH
 * @brief 补丁号
 * 
 * 当前值：0
 * 含义：API的补丁号，当进行向后兼容的问题修复时递增
 */
#define WOAN_API_VERSION_PATCH 0

/**
 * @def WOAN_API_VERSION_STR
 * @brief 版本字符串
 * 
 * 当前值："1.0.0"
 * 格式："{MAJOR}.{MINOR}.{PATCH}"
 * 用途：用于显示和日志记录的完整版本字符串
 */
#define WOAN_API_VERSION_STR "1.0.0"

/**
 * @brief 获取版本主版本号
 * @return 主版本号
 */
WOAN_C_API int woan_api_get_version_major(void);

/**
 * @brief 获取版本次版本号
 * @return 次版本号
 */
WOAN_C_API int woan_api_get_version_minor(void);

/**
 * @brief 获取版本补丁号
 * @return 补丁号
 */
WOAN_C_API int woan_api_get_version_patch(void);

/**
 * @brief 获取版本字符串
 * @return 版本字符串指针（格式："major.minor.patch"）
 */
WOAN_C_API const char* woan_api_get_version_string(void);

#ifdef __cplusplus
}
#endif