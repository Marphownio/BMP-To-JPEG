/** 
 * @file cjpeg.c
 * @brief main file, convert BMP to JPEG image.
 */

#include "cjpeg.h"
#include "cio.h"

/* YCbCr to RGB transformation */

/*
 * precalculated tables for a faster YCbCr->RGB transformation.
 * use a INT32 table because we'll scale values by 2^16 and
 * work with integers.
 */

ycbcr_tables ycc_tables;


//初始化ycrcb表，以便于将rgb色彩空间转化为ycrcb色彩空间
void
init_ycbcr_tables()
{
    UINT16 i;
    for (i = 0; i < 256; i++) {//8位深的色彩，i.e.0~255
        ycc_tables.r2y[i]  = (INT32)(65536 *  0.299   + 0.5) * i;
        ycc_tables.r2cb[i] = (INT32)(65536 * -0.16874 + 0.5) * i;
        ycc_tables.r2cr[i] = (INT32)(32768) * i;
        ycc_tables.g2y[i]  = (INT32)(65536 *  0.587   + 0.5) * i;
        ycc_tables.g2cb[i] = (INT32)(65536 * -0.33126 + 0.5) * i;
        ycc_tables.g2cr[i] = (INT32)(65536 * -0.41869 + 0.5) * i;
        ycc_tables.b2y[i]  = (INT32)(65536 *  0.114   + 0.5) * i;
        ycc_tables.b2cb[i] = (INT32)(32768) * i;
        ycc_tables.b2cr[i] = (INT32)(65536 * -0.08131 + 0.5) * i;
    }
}

//对每一个DCT单元（8X8），进行rgb到ycrcb的色彩转化
void
rgb_to_ycbcr(UINT8 *rgb_unit, ycbcr_unit *ycc_unit, int x, int w)
{
    //x：该DCTunit的横坐标  w:该图像的最大宽度（以免越界）
    ycbcr_tables *tbl = &ycc_tables;
    UINT8 r, g, b;
    #ifdef REVERSED
        int src_pos = (x + w * (DCTSIZE - 1)) * 3;//从最后一行的最左边开始
    #else
        int src_pos = x * 3;//一个像素点中存储了RGB三个颜色分量，所以乘3
    #endif
    int dst_pos = 0;
    int i, j;
    for (j = 0; j < DCTSIZE; j++) {
        for (i = 0; i < DCTSIZE; i++) {
            b = rgb_unit[src_pos];
            g = rgb_unit[src_pos+1];
            r = rgb_unit[src_pos+2];
            ycc_unit->y[dst_pos] = (INT8) ((UINT8)
                ((tbl->r2y[r] + tbl->g2y[g] + tbl->b2y[b]) >> 16) - 128);
            ycc_unit->cb[dst_pos] = (INT8) ((UINT8)
                ((tbl->r2cb[r] + tbl->g2cb[g] + tbl->b2cb[b]) >> 16));
            ycc_unit->cr[dst_pos] = (INT8) ((UINT8)
                ((tbl->r2cr[r] + tbl->g2cr[g] + tbl->b2cr[b]) >> 16));
            src_pos += 3;
            dst_pos++;
        }
        #ifdef REVERSED
            src_pos -= (w + DCTSIZE) * 3;//进入该unit的下(上)一行的最左边进行转化（二维转一维）
        #else
            src_pos += (w - DCTSIZE) * 3;
        #endif
        //src_pos += (w - DCTSIZE) * 3;
    }
}


/* quantization */

quant_tables q_tables;

void
init_quant_tables(UINT32 scale_factor)
{
    quant_tables *tbl = &q_tables;
    int temp1, temp2;
    int i;
    for (i = 0; i < DCTSIZE2; i++) {
        temp1 = ((UINT32) STD_LU_QTABLE[i] * scale_factor + 50) / 100;
        if (temp1 < 1)
            temp1 = 1;
        if (temp1 > 255)
            temp1 = 255;
        tbl->lu[ZIGZAG[i]] = (UINT8) temp1;

        temp2 = ((UINT32) STD_CH_QTABLE[i] * scale_factor + 50) / 100;
        if (temp2 < 1)
            temp2 = 1;
        if (temp2 > 255)
            temp2 = 255;
        tbl->ch[ZIGZAG[i]] = (UINT8) temp2;
    }
}

void
jpeg_quant(ycbcr_unit *ycc_unit, quant_unit *q_unit)
{
    quant_tables *tbl = &q_tables;
    float q_lu, q_ch;
    int x, y, i = 0;
    for (x = 0; x < DCTSIZE; x++) {
        for (y = 0; y < DCTSIZE; y++) {
            q_lu = 1.0 / ((double) tbl->lu[ZIGZAG[i]] * \
                    AAN_SCALE_FACTOR[x] * AAN_SCALE_FACTOR[y] * 8.0);           //得到y分量的量化表每个值的倒数，加入了缩放因子AAN_SCALE_FACTOR
            q_ch = 1.0 / ((double) tbl->ch[ZIGZAG[i]] * \
                    AAN_SCALE_FACTOR[x] * AAN_SCALE_FACTOR[y] * 8.0);          //得到cr、cb分量的量化表每个值的倒数，加入了缩放因子AAN_SCALE_FACTOR

            q_unit->y[i] = (INT16)(ycc_unit->y[i]*q_lu + 16384.5) - 16384;      //对每个y值除以量化数值后，四舍五入
            q_unit->cb[i] = (INT16)(ycc_unit->cb[i]*q_ch + 16384.5) - 16384;    //对每个cr值除以量化数值后，四舍五入
            q_unit->cr[i] = (INT16)(ycc_unit->cr[i]*q_ch + 16384.5) - 16384;    //对每个cb值除以量化数值后，四舍五入

            i++;
        }
    }
}


/* huffman compression */

huff_tables h_tables;

void
set_huff_table(UINT8 *nrcodes, UINT8 *values, BITS *h_table)
{
    int i, j, k;
    j = 0;
    UINT16 value = 0;
    for (i = 1; i <= 16; i++) {
        for (k = 0; k < nrcodes[i]; k++) {
            h_table[values[j]].len = i;
            h_table[values[j]].val = value;
            j++;
            value++;
        }
        value <<= 1;
    }
}

void
init_huff_tables()
{
    huff_tables *tbl = &h_tables;
    set_huff_table(STD_LU_DC_NRCODES, STD_LU_DC_VALUES, tbl->lu_dc);
    set_huff_table(STD_LU_AC_NRCODES, STD_LU_AC_VALUES, tbl->lu_ac);
    set_huff_table(STD_CH_DC_NRCODES, STD_CH_DC_VALUES, tbl->ch_dc);
    set_huff_table(STD_CH_AC_NRCODES, STD_CH_AC_VALUES, tbl->ch_ac);
}

void
set_bits(BITS *bits, INT16 data)
{
    /******************************************************/
    /*                                                    */
    /*             finish the missing codes               */
    /*                                                    */
    /******************************************************/
}

#ifdef DEBUG
void
print_bits(BITS bits)
{
    printf("%hu %hu\t", bits.len, bits.val);
}
#endif

/*
 * compress JPEG
 */
void
jpeg_compress(compress_io *cio,
        INT16 *data, INT16 *dc, BITS *dc_htable, BITS *ac_htable)
{
    INT16 zigzag_data[DCTSIZE2];
    BITS bits;
    INT16 diff;
    int i, j;
    int zero_num;
    int mark;

    /*进行z字形编码，存储到zigzag_data数组中 i.e. zigzag_data数组按照z字形存储了data中的数据*/
    for (i = 0; i < DCTSIZE2; i++)
        zigzag_data[ZIGZAG[i]] = data[i];

    /* 写入直流分量，直流系数利用差值编码*/
    diff = zigzag_data[0] - *dc;
    *dc = zigzag_data[0];
    
    if (diff == 0)//如果与上一个block的直流分量一致，写入直流哈夫曼表中的0
        write_bits(cio, dc_htable[0]);
    else {
        set_bits(&bits, diff);//如果与上一个block的直流分量不一样
        write_bits(cio, dc_htable[bits.len]);//写入哈夫曼直流类号
        write_bits(cio, bits); //写入哈夫曼直流数据号（负数则取反码）
    }

    /* 写入交流分量，直流系数利用差值编码 */
    int end = DCTSIZE2 - 1;   //终点
    while (zigzag_data[end] == 0 && end > 0)
        end--;                //找到最后一个非0的交流分量
    for (i = 1; i <= end; i++) {
        j = i;
        //如果找到了0
        while (zigzag_data[j] == 0 && j <= end)
            j++;
        zero_num = j - i;     //当前值为0的交流分量个数
        for (mark = 0; mark < zero_num / 16; mark++)
            write_bits(cio, ac_htable[0xF0]);  //mark个（16个0），是第0xf0类
        zero_num = zero_num % 16;
        set_bits(&bits, zigzag_data[j]);  //获取非0值zigzag_data[j]的长度、值
        write_bits(cio, ac_htable[zero_num * 16 + bits.len]);//zero_num * 16：非0值前0的个数，bits.len非零值的类
        write_bits(cio, bits);//写入非0值
        i = j;
    }

    /* write end of unit */
    if (end != DCTSIZE2 - 1)
        write_bits(cio, ac_htable[0]);//如果unit最后一个不是非0值，则用EOB结束
}


/*
 * main JPEG encoding
 */
void
jpeg_encode(compress_io *cio, bmp_info *binfo)
{
    /*初始化ycrcb表、量化表、哈夫曼表*/
    UINT32 scale = 50;
    init_ycbcr_tables();
    init_quant_tables(scale);
    init_huff_tables();

    /* write info */
    write_file_header(cio);
    write_frame_header(cio, binfo);
    write_scan_header(cio);

    /* encode */
    mem_mgr *in = cio->in;
    ycbcr_unit ycc_unit;
    quant_unit q_unit;

    INT16 dc_y = 0,
            dc_cb = 0,
            dc_cr = 0;
    int x, y;

    //通常来说bmp图像的存储都是倒过来的，因此我们需要将起始位置与结束位置调换
#ifdef REVERSED
    int in_size = in->end - in->set;
    int offset = binfo->offset + (binfo->datasize/in_size - 1) * in_size;
#else
    int offset = binfo->offset;
#endif


    fseek(in->fp, offset, SEEK_SET);

    /***************************代码补充部分*****************************/
    for(i = 0 ; i < binfo->height ; i += 8){       //遍历bmp图像的高

        //i+=8是因为一个DCTunit的大小是8X8
        for(j = o ; j < binfo->width ; j += 8){    //遍历bmp图像的宽
            
            //1、RGB颜色空间转换至YCbCr色彩空间
            rgb_to_ycbcr(in->set, ycc_unit, j, binfo->width);
            
            //2、对3个分量分别进行向前DCT离散余弦变化
            jpeg_fdct(ycc_unit.y);
            jpeg_fdct(ycc_unit.cb);
            jpeg_fdct(ycc_unit.cr);     

            //3、量化
            jpeg_quant(ycc_unit, q_unit);

            //4、对每个分量分别进行huffman压缩编码
            jpeg_compress(cio, q_unit.y, &dc_y, h_tables.lu_dc, h_tables.lu_ac);
            jpeg_compress(cio, q_unit.cb, &dc_cb, h_tables.ch_dc, h_tables.ch_ac);
            jpeg_compress(cio, q_unit.cr, &dc_cr, h_tables.ch_dc, h_tables.ch_ac);
        }
    }

    /*************************代码补充部分结束**************************/

    write_align_bits(cio);

    /* write file end */
    write_file_trailer(cio);
}



bool
is_bmp(FILE *fp)
{
    UINT8 marker[3];
    if (fread(marker, sizeof(UINT16), 2, fp) != 2)
        err_exit(FILE_READ_ERR);
    if (marker[0] != 0x42 || marker[1] != 0x4D)
        return false;
    rewind(fp);
    return true;
}

void
err_exit(const char *error_string, int exit_num)
{
    printf(error_string);
    exit(exit_num);
}


void
print_help()
{
    printf("compress BMP file into JPEG file.\n");
    printf("Usage:\n");
    printf("    cjpeg {BMP} {JPEG}\n");
    printf("\n");
    printf("Author: Yu, Le <yeolar@gmail.com>\n");
}



int
main(int argc, char *argv[])
{
    if (argc == 3) {
        /* open bmp file */
        FILE *bmp_fp = fopen(argv[1], "rb");
        if (!bmp_fp)
            err_exit(FILE_OPEN_ERR);
        if (!is_bmp(bmp_fp))
            err_exit(FILE_TYPE_ERR);

        /* open jpeg file */
        FILE *jpeg_fp = fopen(argv[2], "wb");
        if (!jpeg_fp)
            err_exit(FILE_OPEN_ERR);

        /* get bmp info */
        bmp_info binfo;
        read_bmp(bmp_fp, &binfo);

        /* init memory for input and output */
        compress_io cio;
        int in_size = (binfo.width * 3 + 3) / 4 * 4 * DCTSIZE;
        int out_size = MEM_OUT_SIZE;
        init_mem(&cio, bmp_fp, in_size, jpeg_fp, out_size);

        /* main encode process */
        jpeg_encode(&cio, &binfo);

        /* flush and free memory, close files */
        if (! (cio.out->flush_buffer) (&cio))
            err_exit(BUFFER_WRITE_ERR);
        free_mem(&cio);
        fclose(bmp_fp);
        fclose(jpeg_fp);
    }
    else
        print_help();
    exit(0);
}
