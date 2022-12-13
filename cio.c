/** 
 * @file cio.c
 * @brief memory manager and operations for compressing JPEG IO.
 */

#include <string.h>
#include "cjpeg.h"
#include "cio.h"


/*
 * flush input and output of compress IO.
 */


bool
flush_cin_buffer(void *cio)
{
    mem_mgr *in = ((compress_io *) cio)->in;
    size_t len = in->end - in->set;
    memset(in->set, 0, len);
    if (fread(in->set, sizeof(UINT8), len, in->fp) != len)
        return false;
    #ifdef REVERSED
        fseek(in->fp, -len * 2, SEEK_CUR);
    #endif
        in->pos = in->set;
    return true;
}

bool
flush_cout_buffer(void *cio)
{
    mem_mgr *out = ((compress_io *) cio)->out;
    size_t len = out->pos - out->set;
    if (fwrite(out->set, sizeof(UINT8), len, out->fp) != len)
        return false;
    memset(out->set, 0, len);
    out->pos = out->set;
    return true;
}


/*
 * init memory manager.
 */

void
init_mem(compress_io *cio,
         FILE *in_fp, int in_size, FILE *out_fp, int out_size)
{
    cio->in = (mem_mgr *) malloc(sizeof(mem_mgr));
    if (!cio->in)
        err_exit(BUFFER_ALLOC_ERR);
    cio->in->set = (UINT8 *) malloc(sizeof(UINT8) * in_size);
    if (!cio->in->set)
        err_exit(BUFFER_ALLOC_ERR);
    cio->in->pos = cio->in->set;
    cio->in->end = cio->in->set + in_size;
    cio->in->flush_buffer = flush_cin_buffer;
    cio->in->fp = in_fp;

    cio->out = (mem_mgr *) malloc(sizeof(mem_mgr));
    if (!cio->out)
        err_exit(BUFFER_ALLOC_ERR);
    cio->out->set = (UINT8 *) malloc(sizeof(UINT8) * out_size);
    if (!cio->out->set)
        err_exit(BUFFER_ALLOC_ERR);
    cio->out->pos = cio->out->set;
    cio->out->end = cio->out->set + out_size;
    cio->out->flush_buffer = flush_cout_buffer;
    cio->out->fp = out_fp;

    cio->temp_bits.len = 0;
    cio->temp_bits.val = 0;
}

void
free_mem(compress_io *cio)
{
    fflush(cio->out->fp);
    free(cio->in->set);
    free(cio->out->set);
    free(cio->in);
    free(cio->out);
}


/*
 * write operations.
 */

void
write_byte(compress_io *cio, UINT8 val)
{
    mem_mgr *out = cio->out;
    *(out->pos)++ = val & 0xFF;
    if (out->pos == out->end) {
        if (!(out->flush_buffer)(cio))
            err_exit(BUFFER_WRITE_ERR);
    }
}

void
write_word(compress_io *cio, UINT16 val)
{
    write_byte(cio, (val >> 8) & 0xFF);
    write_byte(cio, val & 0xFF);
}

void
write_marker(compress_io *cio, JPEG_MARKER mark)
{
    write_byte(cio, 0xFF);
    write_byte(cio, (int) mark);
}

void
write_bits(compress_io *cio, BITS bits)
{
    /************ finish the missing codes *****************/
    //write_bits()实现思路：
    //为了充分利用存储空间，我们需要将数据流按照字（16bits）写入。
    //但是每次交付的数据难免有凑不够16bits的情况，因此我们需要将
    //超出但不够16bits的部分数据保存下来，留到下一次数据来时拼接为
    //新的字写入。
    //其中，temp->val存储的是temp->len位上次写入之后剩下的数据，
    //temp->len是实际的位数，存储在temp->val中的时候末尾已经补0至16位了
    //实现如下：
    
    //获取上次遗留的数据
    BITS *temp = &(cio->temp_bits);
    UINT16 word;
    UINT8 byte1, byte2;
    //计算新数据与遗留数据的长度之和
    int len = bits.len + temp->len - 16;
    if (len >= 0) {//如果遗留数据和新数据的长度之大于等于16bits
        //将新数据拼接到遗留数据之后，超出16bit的部分暂时丢弃
        word = temp->val | bits.val >> len;
        //获取16bit的前8个bit
        byte1 = word >> 8;
        //作为一个byte写入
        write_byte(cio, byte1);
        //如果刚刚写入的byte是0xff，代表是图像数据的开始
        if (byte1 == 0xFF)
            //故写入0x00代表图像数据开始
            write_byte(cio, 0);
        //获取word的后8个bits    
        byte2 = word & 0xFF;
        //将后8个bits写入
        write_byte(cio, byte2);
        //如果写入的byte是0xff，代表是图像数据的开始
        if (byte2 == 0xFF)
            //故写入0x00代表图像数据开始
            write_byte(cio, 0);
        //更新遗留数据的长度
        temp->len = len;
        //保存新的遗留数据
        temp->val = bits.val << (16 - len);
    }
    else {
        //如果遗留数据和新数据的长度之和不到16bits
        //获得真实的长度和，保存在temp->len中，等待下一次的写入
        temp->len = 16 + len;            
        //将新数据拼接到遗留数据之后，然后末尾补0，补到16位，等待下一次的数据写入
        temp->val |= bits.val << -len;   
    }
    //要求按byte写入，但是为什么是一次选择16位的数据写入呢？
    //因为：在JPEG文件格式中，一个字（16位）的存储使用的是
    //Motorola 格式，而不是 Intel 格式。即高字节（高8
    //位）在数据流的前面，低字节（低8位）在数据流的后面
    /******************************************************/
}

void
write_align_bits(compress_io *cio)
{
    /*************finish the missing codes*****************/
    //获取上一次遗留下来的数据
    BITS *temp = &(cio->temp_bits);
    BITS align_bits;
    //获取需要补1的bit的长度，保存在align_bits.len中
    align_bits.len = 8 - temp->len % 8;
    //将align_bits.len个1保存在align_bits.val中
    align_bits.val = (UINT16) ~0x0 >> temp->len % 8;
    UINT8 byte;
    //将align_bits调用write_bits()函数写入
    write_bits(cio, align_bits);
    //如果刚好补充了8个1，即0xff的情况
    //需要避免程序将这个认作图像数据的开始
    //因此将temp->val补充在1111_1111的后面
    if (temp->len == 8) {
        byte = temp->val >> 8;
        write_byte(cio, byte);
        //如果temp->val本就是0xff，那么代表图像数据的开始，需要补充0x00
        if (byte == 0xFF)
            write_byte(cio, 0);
    }
    /******************************************************/
}

