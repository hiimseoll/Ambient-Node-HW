#!/usr/bin/env python3
import pcbnew
import sys

def create_perfboard(board, width_mm, height_mm, grid_mm=2.54, start_x=10, start_y=10):
    """
    만능기판 패드 생성
    
    Args:
        board: PCB 보드 객체
        width_mm: 폭 (mm)
        height_mm: 높이 (mm)
        grid_mm: 격자 간격 (기본 2.54mm)
        start_x: 시작 X 위치
        start_y: 시작 Y 위치
    """
    x_count = int(width_mm / grid_mm)
    y_count = int(height_mm / grid_mm)
    
    print(f"생성 중: {x_count}x{y_count} = {x_count * y_count}개 패드")
    
    for x in range(x_count):
        for y in range(y_count):
            # Footprint 생성
            module = pcbnew.FOOTPRINT(board)
            module.SetReference(f"H{x}_{y}")
            board.Add(module)
            
            # 패드 생성
            pad = pcbnew.PAD(module)
            pad.SetSize(pcbnew.VECTOR2I(
                pcbnew.FromMM(1.6), 
                pcbnew.FromMM(1.6)
            ))
            pad.SetDrillSize(pcbnew.VECTOR2I(
                pcbnew.FromMM(1.0), 
                pcbnew.FromMM(1.0)
            ))
            pad.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
            pad.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
            pad.SetPosition(pcbnew.VECTOR2I(
                pcbnew.FromMM(start_x + x * grid_mm),
                pcbnew.FromMM(start_y + y * grid_mm)
            ))
            
            # 레이어 설정
            lset = pcbnew.LSET()
            lset.AddLayer(pcbnew.F_Cu)
            lset.AddLayer(pcbnew.B_Cu)
            lset.AddLayer(pcbnew.F_Mask)
            lset.AddLayer(pcbnew.B_Mask)
            pad.SetLayerSet(lset)
            
            pad.SetPadName("1")
            module.Add(pad)
    
    print("완료!")
    return x_count * y_count

# 메인 실행 (PCB Editor에서 실행 시)
if __name__ == "__main__":
    board = pcbnew.GetBoard()
    
    # 설정값
    WIDTH = 50    # mm
    HEIGHT = 70   # mm
    GRID = 2.54   # mm
    
    count = create_perfboard(board, WIDTH, HEIGHT, GRID)
    pcbnew.Refresh()
