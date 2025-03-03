#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <unistd.h>
#include <ros/ros.h>
#include <termio.h>
#include "std_msgs/String.h"

#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"

#define FRAME_LEN 640
#define BUFFER_SIZE 4096
#define SAMPLE_RATE_16K (16000)
#define SAMPLE_RATE_8K (8000)
#define MAX_GRAMMARID_LEN (32)
#define MAX_PARAMS_LEN (1024)

#define pub_min_v 0.3


char asr_text[100];
float asr_confidence = -1.0;

const char *ASR_RES_PATH = "fo|res/asr/common.jet"; // 离线语法识别资源路径
const char *GRM_BUILD_PATH = "res/asr/GrmBuilld";   // 构建离线语法识别网络生成数据保存路径
const char *GRM_FILE = "call.bnf";                  // 构建离线识别语法网络所用的语法文件
const char *LEX_NAME = "contact";                   // 更新离线识别语法的contact槽（语法文件为此示例中使用的call.bnf）

typedef struct _UserData
{
    int build_fini;                     // 标识语法构建是否完成
    int update_fini;                    // 标识更新词典是否完成
    int errcode;                        // 记录语法构建或更新词典回调错误码
    char grammar_id[MAX_GRAMMARID_LEN]; // 保存语法构建返回的语法ID
} UserData;

// const char *get_audio_file(void);    // 选择进行离线语法识别的语音文件
int build_grammar(UserData *udata); // 构建离线识别语法网络
// int update_lexicon(UserData *udata); // 更新离线识别语法词典
int run_asr(UserData *udata); // 进行离线语法识别


int build_grm_cb(int ecode, const char *info, void *udata)
{
    UserData *grm_data = (UserData *)udata;

    if (NULL != grm_data)
    {
        grm_data->build_fini = 1;
        grm_data->errcode = ecode;
    }

    if (MSP_SUCCESS == ecode && NULL != info)
    {
        printf("构建语法成功！ 语法ID:%s\n", info);
        if (NULL != grm_data)
            snprintf(grm_data->grammar_id, MAX_GRAMMARID_LEN - 1, info);
    }
    else
        printf("构建语法失败！%d\n", ecode);

    return 0;
}

int build_grammar(UserData *udata)
{
    FILE *grm_file = NULL;
    char *grm_content = NULL;
    unsigned int grm_cnt_len = 0;
    char grm_build_params[MAX_PARAMS_LEN] = {NULL};
    int ret = 0;

    grm_file = fopen(GRM_FILE, "rb");
    if (NULL == grm_file)
    {
        printf("打开\"%s\"文件失败！[%s]\n", GRM_FILE, strerror(errno));
        return -1;
    }

    fseek(grm_file, 0, SEEK_END);
    grm_cnt_len = ftell(grm_file);
    fseek(grm_file, 0, SEEK_SET);

    grm_content = (char *)malloc(grm_cnt_len + 1);
    if (NULL == grm_content)
    {
        printf("内存分配失败!\n");
        fclose(grm_file);
        grm_file = NULL;
        return -1;
    }
    fread((void *)grm_content, 1, grm_cnt_len, grm_file);
    grm_content[grm_cnt_len] = '\0';
    fclose(grm_file);
    grm_file = NULL;

    snprintf(grm_build_params, MAX_PARAMS_LEN - 1,
             "engine_type = local, \
		asr_res_path = %s, sample_rate = %d, \
		grm_build_path = %s, ",
             ASR_RES_PATH,
             SAMPLE_RATE_16K,
             GRM_BUILD_PATH);
    ret = QISRBuildGrammar("bnf", grm_content, grm_cnt_len, grm_build_params, build_grm_cb, udata);

    free(grm_content);
    grm_content = NULL;

    return ret;
}

int update_lex_cb(int ecode, const char *info, void *udata)
{
    UserData *lex_data = (UserData *)udata;

    if (NULL != lex_data)
    {
        lex_data->update_fini = 1;
        lex_data->errcode = ecode;
    }

    if (MSP_SUCCESS == ecode)
        printf("更新词典成功！\n");
    else
        printf("更新词典失败！%d\n", ecode);

    return 0;
}


static void show_result(char *string, char is_over)
{
    const char *input_tag = "input=";
    const char *confidence_tag = "confidence=";
    char *input_start,*input_end, *confidence_start;
    char result_text[100]; // 用于存储格式化后的结果文本

    // 查找输入文本的开始位置
    input_start = strstr(string, input_tag);
    if (input_start)
    {
        // 跳过标签获取实际的输入文本
        input_start += strlen(input_tag);
        input_end = strchr(input_start, ' ');
        if (input_end == NULL)
        {
            // 如果没有空格，则取到字符串末尾
            input_end = input_start + strlen(input_start);
        }
    }
    // 复制input=后面的文本到全局变量
    int len = input_end - input_start;
    if (len > sizeof(asr_text) - 1)
    {
        len = sizeof(asr_text) - 1;
    }
    strncpy(asr_text, input_start, len);
    asr_text[len] = '\0'; // 确保字符串以空字符结尾

    // 查找置信度的开始位置
    confidence_start = strstr(string, confidence_tag);
    if (confidence_start)
    {
        // 跳过标签并提取置信度数值
        confidence_start += strlen(confidence_tag);
        int confidence_int = atoi(confidence_start); // 转换为整数
        float confidence = confidence_int / 100.0f;  // 除以100转换为浮点数
        asr_confidence = confidence_int / 100.0f;
        // 构造新的格式化字符串
        snprintf(result_text, sizeof(result_text), "Result_text：%s \tConfidence：%.2f",
                 input_start ? input_start : "未知", confidence);
    }

    // 输出结果
    printf("%s\n", result_text);

    //putchar('\n');
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
    if (result)
    {
        size_t left = g_buffersize - 1 - strlen(g_result);
        size_t size = strlen(result);
        if (left < size)
        {
            g_result = (char *)realloc(g_result, g_buffersize + BUFFER_SIZE);
            if (g_result)
                g_buffersize += BUFFER_SIZE;
            else
            {
                printf("mem alloc failed\n");
                return;
            }
        }
        strncat(g_result, result, size);
        show_result(g_result, is_last);
    }
}
void on_speech_begin()
{
    if (g_result)
    {
        free(g_result);
    }
    g_result = (char *)malloc(BUFFER_SIZE);
    g_buffersize = BUFFER_SIZE;
    memset(g_result, 0, g_buffersize);

    printf("\nStart Listening...\n");
}
void on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT)
        printf("Speaking done \n");
    else
        printf("\nRecognizer error %d\n", reason);
}

/* demo recognize the audio from microphone */
static void demo_mic(const char *session_begin_params)
{
    int errcode;
    int i = 0;

    struct speech_rec iat;

    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end};

    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
    if (errcode)
    {
        printf("speech recognizer init failed\n");
        return;
    }
    errcode = sr_start_listening(&iat);
    if (errcode)
    {
        printf("start listen failed %d\n", errcode);
    }

    
     while (1)
     {
         if (getchar() == 32)
        {
             printf("Speaking done .\n");
            break;
         }
     }

    errcode = sr_stop_listening(&iat);
    if (errcode)
    {
        printf("stop listening failed %d\n", errcode);
    }

    sr_uninit(&iat);
}

int run_asr(UserData *udata)
{
    char asr_params[MAX_PARAMS_LEN] = {NULL};
    const char *rec_rslt = NULL;
    const char *session_id = NULL;
    // const char *asr_audiof = NULL;
    FILE *f_pcm = NULL;
    char *pcm_data = NULL;
    long pcm_count = 0;
    long pcm_size = 0;
    int last_audio = 0;

    int aud_stat = MSP_AUDIO_SAMPLE_CONTINUE;
    int ep_status = MSP_EP_LOOKING_FOR_SPEECH;
    int rec_status = MSP_REC_STATUS_INCOMPLETE;
    int rss_status = MSP_REC_STATUS_INCOMPLETE;
    int errcode = -1;
    int aud_src = 0;
    // 离线语法识别参数设置
    snprintf(asr_params, MAX_PARAMS_LEN - 1,
             "engine_type = local, \
		asr_res_path = %s, sample_rate = %d, \
		grm_build_path = %s, local_grammar = %s, \
		result_type = plain, result_encoding = UTF-8, ",
             ASR_RES_PATH,
             SAMPLE_RATE_16K,
             GRM_BUILD_PATH,
             udata->grammar_id);

    demo_mic(asr_params);
  
    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "asr_1");
    ros::NodeHandle n;
    ros::Publisher asr_text_pub = n.advertise<std_msgs::String>("/iat", 10);
    ros::Rate loop_rate(1);

    //终端设置
     termios tms_old, tms_new;
     tcgetattr(0, &tms_old);
     tms_new = tms_old;
     tms_new.c_lflag &= ~(ICANON | ECHO);
     tcsetattr(0, TCSANOW, &tms_new);

    const char *login_config = "appid = 9f9d795d"; // 登录参数
    UserData asr_data;
    int ret = 0;
    char c;

    ret = MSPLogin(NULL, NULL, login_config); // 第一个参数为用户名，第二个参数为密码，传NULL即可，第三个参数是登录参数
    if (MSP_SUCCESS != ret)
    {
        printf("登录失败：%d\n", ret);
        goto exit;
    }

    memset(&asr_data, 0, sizeof(UserData));

    printf("构建离线识别语法网络...\n");
    ret = build_grammar(&asr_data); // 第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建
                                    // while (1)
                                    // {

    if (MSP_SUCCESS != ret)
    {
        printf("构建语法调用失败！\n");
        goto exit;
    }
    while (1 != asr_data.build_fini)
        usleep(300 * 1000);
    if (MSP_SUCCESS != asr_data.errcode)
        goto exit;
    printf("离线识别语法网络构建完成，按下'Space'开始识别...\n");

    //ROS_INFO("\nPress \"Space\" key to Start,Press \"Enter\" key to Exit.");

    int key_input;
    while (1)
    {
        key_input = getchar();
        if (key_input == 32)
        {
            ret = run_asr(&asr_data);
            if (MSP_SUCCESS != ret)
            {
                printf("离线语法识别出错: %d \n", ret);
                goto exit;
            }
        }
        else if (key_input == 10)
        {
            ROS_INFO("Node Exit.");
            break;
        }

        if (asr_confidence >= pub_min_v )
        {
            std_msgs::String message;
            message.data = asr_text;
            asr_text_pub.publish(message);
            asr_confidence = -1.0 ; //置零
        }
        else if (asr_confidence < pub_min_v && asr_confidence >= 0.0)
        {
            printf("Low confidence ! Commands will not publish !\n");
            asr_confidence = -1.0; // 置零
        }
        
        printf("Press \"Space\" key to Start,Press \"Enter\" key to Exit.\n");
        loop_rate.sleep();
    }

exit:
    tcsetattr(0, TCSANOW, &tms_old);
    MSPLogout();
    return 0;
}

