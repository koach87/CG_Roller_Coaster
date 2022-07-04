Project3 - RollerCoaster

技術：
Curve切法：代入curve matrix，求得線段對應點。

Arc Length Parameterization：因點和點之間的距離會不相同，因此需要做arcLength處理。累積路線的距離，到達一定的長度後設立點以確保每個點的距離相同。

Simple Physics：本project實作重力效果，當y向量向上時，降低速度，反之亦然。

Non-flat terrain：利用opencv讀取height map 檔案，讀取檔案內的灰階值作為高度。

使用方法：打開RollerCoasters.exe即可使用
 
 

