#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
批量替换 CvMat 相关代码为 OpenCV 4.x 兼容的 cv::Mat
"""

import re
import os

def fix_cvmat_in_file(file_path):
    """修复单个文件中的 CvMat 相关代码"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    original_content = content
    
    # 1. 替换函数参数中的 CvMat * -> cv::Mat&
    # 注意：const CvMat * 需要特殊处理
    content = re.sub(r'const CvMat\s*\*\s*(\w+)', r'const cv::Mat& \1', content)
    content = re.sub(r'CvMat\s*\*\s*(\w+)', r'cv::Mat& \1', content)
    
    # 2. 替换 cvCreateMat -> cv::Mat 构造函数
    # cvCreateMat(rows, cols, CV_64F) -> cv::Mat(rows, cols, CV_64F)
    content = re.sub(
        r'CvMat\s*\*\s*(\w+)\s*=\s*cvCreateMat\(([^,]+),\s*([^,]+),\s*CV_64F\)',
        r'cv::Mat \1(\2, \3, CV_64F)',
        content
    )
    
    # 3. 替换 cvMat 宏 -> cv::Mat 构造函数
    # CvMat name = cvMat(rows, cols, CV_64F, data) -> cv::Mat name(rows, cols, CV_64F, data)
    content = re.sub(
        r'CvMat\s+(\w+)\s*=\s*cvMat\(([^,]+),\s*([^,]+),\s*CV_64F,\s*([^)]+)\)',
        r'cv::Mat \1(\2, \3, CV_64F, \4)',
        content
    )
    
    # 4. 删除 cvReleaseMat 调用
    content = re.sub(r'\s*cvReleaseMat\(&[^)]+\);\s*\n', '', content)
    
    # 5. 替换 cvMulTransposed -> cv::mulTransposed
    # cvMulTransposed(M, &MtM, 1) -> cv::mulTransposed(M, MtM, 1, false, -1)
    content = re.sub(
        r'cvMulTransposed\(([^,]+),\s*&([^,]+),\s*1\)',
        r'cv::mulTransposed(\1, \2, true, cv::noArray(), -1, CV_64F)',
        content
    )
    
    # 6. 替换 cvSVD -> cv::SVD
    # cvSVD(&MtM, &D, &Ut, 0, flags) -> cv::SVD::compute(MtM, D, Ut, cv::SVD::MODIFY_A | cv::SVD::U_T)
    content = re.sub(
        r'cvSVD\(&([^,]+),\s*&([^,]+),\s*&([^,]+),\s*0,\s*([^)]+)\)',
        r'cv::SVD::compute(\1, \2, \3, \4)',
        content
    )
    # 处理 CV_SVD_MODIFY_A | CV_SVD_U_T
    content = re.sub(r'CV_SVD_MODIFY_A\s*\|\s*CV_SVD_U_T', r'cv::SVD::MODIFY_A | cv::SVD::U_T', content)
    content = re.sub(r'CV_SVD', r'cv::SVD', content)
    
    # 7. 替换 cvSolve -> cv::solve
    # cvSolve(&L_6x4, Rho, &B4, CV_SVD) -> cv::solve(L_6x4, Rho, B4, cv::DECOMP_SVD)
    content = re.sub(
        r'cvSolve\(&([^,]+),\s*([^,]+),\s*&([^,]+),\s*CV_SVD\)',
        r'cv::solve(\1, \2, \3, cv::DECOMP_SVD)',
        content
    )
    
    # 8. 替换 cvmSet -> 直接访问 Mat 元素
    # cvmSet(&L_6x4, i, 0, value) -> L_6x4.at<double>(i, 0) = value
    content = re.sub(
        r'cvmSet\(&([^,]+),\s*([^,]+),\s*([^,]+),\s*([^)]+)\)',
        r'\1.at<double>(\2, \3) = \4',
        content
    )
    
    # 9. 替换 cvmGet -> 直接访问 Mat 元素
    # cvmGet(L_6x10, i, 0) -> L_6x10.at<double>(i, 0)
    content = re.sub(
        r'cvmGet\(([^,]+),\s*([^,]+),\s*([^)]+)\)',
        r'\1.at<double>(\2, \3)',
        content
    )
    
    # 10. 替换 M->data.db[...] -> M.ptr<double>(row)[col] 或 M.at<double>(row, col)
    # 这个比较复杂，需要根据上下文判断
    # 先处理简单的：M->data.db[index] -> M.ptr<double>()[index]
    content = re.sub(
        r'(\w+)->data\.db\[([^\]]+)\]',
        r'\1.ptr<double>()[\2]',
        content
    )
    
    # 11. 替换函数调用中的 &M -> M (因为现在 M 是引用或值，不是指针)
    # fill_M(&M, ...) -> fill_M(M, ...)
    content = re.sub(r'fill_M\(&([^,)]+)', r'fill_M(\1', content)
    content = re.sub(r'qr_solve\(&([^,)]+)', r'qr_solve(\1', content)
    content = re.sub(r'qr_solve\(([^,]+),\s*&([^,)]+)', r'qr_solve(\1, \2', content)
    content = re.sub(r'qr_solve\(([^,]+),\s*([^,]+),\s*&([^)]+)', r'qr_solve(\1, \2, \3', content)
    
    # 12. 修复 fill_M 函数中的 M->data.db 访问
    # 在 fill_M 函数中，M 现在是引用，需要直接访问
    # M->data.db + row * 12 -> M.ptr<double>(row * 2)
    
    # 13. 修复 compute_A_and_b_gauss_newton 中的参数
    # 函数签名已经改了，但调用处可能还有 & 符号
    
    if content != original_content:
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"已修复: {file_path}")
        return True
    return False

def main():
    """主函数"""
    # 需要修复的文件
    files_to_fix = [
        'src/PnPsolver.cc',
        'include/PnPsolver.h'
    ]
    
    base_dir = '/SDB-8T/mck/ORB_SLAM2_detailed_comments'
    
    for file_path in files_to_fix:
        full_path = os.path.join(base_dir, file_path)
        if os.path.exists(full_path):
            fix_cvmat_in_file(full_path)
        else:
            print(f"文件不存在: {full_path}")
    
    print("\n修复完成！")
    print("注意：可能还需要手动检查一些特殊情况，特别是：")
    print("1. fill_M 函数中的 M->data.db 访问方式")
    print("2. cv::SVD::compute 的参数顺序")
    print("3. cv::mulTransposed 的参数")

if __name__ == '__main__':
    main()
