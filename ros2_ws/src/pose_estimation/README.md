# X-ARM Visual Servoing Project

Este paquete contiene el desarrollo de un sistema de **Visual Servoing** para el brazo robótico XARM, utilizando un sensor Kinect para capturar la nube de puntos de un modelo objetivo y alinearla con un modelo CAD de referencia.

---

## Estructura del Proyecto

* **image\_segmentation.py**
  Nodo ROS2 que aplica segmentación HSV, filtrado de profundidad y operaciones morfológicas para extraer solo la región de interés (pieza impresa) de la imagen RGB y profundidad del Kinect.

* **point\_cloud\_generator.py**
  Nodo ROS2 que convierte la imagen segmentada en una nube de puntos 3D, enriquecida con información de la cámara (intrínsecos) y publica en `/pointcloud/segmented_object`.

* **pose\_estimator.py**
  Nodo ROS2 principal que realiza:

  1. **Publicación del CAD** en `/pointcloud/reference_object`.
  2. **Conversión** de la nube segmentada a Open3D.
  3. **Alineación inicial** por centrado y escalado (sección 3.3.2 de la tesis).
  4. **Registro robusto**: implementación opcional de RANSAC + refinamiento con ICP (secciones 3.3.3 y 3.3.4).
  5. **Estrategia adaptable**: compara RANSAC+ICP vs ICP solo y elige la mejor transformación según un *score* (fitness − RMSE), con umbrales de mejora y número mínimo de inliers para evitar drift.
  6. **Publicación** de la nube alineada en `/pointcloud/aligned_model` y la transformación (`object_frame`) vía TF para controlar el XARM.

---

## Instalación y Uso

1. **Ajustar parámetros** en `pose_estimator.py` (o mediante *launch*):

   * `reference_ply`: Ruta al archivo .ply del modelo CAD.
   * `voxel_size`: Tamaño de voxel para downsampling.
   * `ransac_distance`, `ransac_n`, `ransac_iterations`: Parámetros de RANSAC.
   * `icp_distance`: Umbral de distancia para ICP.
   * `use_ransac`: Habilitar/deshabilitar registro robusto.
   * `min_inliers`, `min_score_delta`: Umbrales para aceptar nuevas poses.

2. **Ejecutar launch**:

   ```bash
   ros2 launch pose_estimation segmentation_point_cloud.launch.py
   ```

---

## Estado Actual

* ✅ Segmentación HSV y filtrado de profundidad funcionando.
* ✅ Generación de nube de puntos segmentada.
* ✅ Alineación inicial por centrado y escalado.
* ✅ Registro robusto con RANSAC + ICP y selección basada en *score*.
* ✅ Publicación de TF de la pose estimada.

**Próximos pasos:**

* Integrar control en bucle de servo para XARM usando el TF publicado.
* Optimización de parámetros dinámicos según cobertura parcial.
* Evaluación de rendimiento en tiempo real y ajuste de thresholds.

---

## Referencias

* Tesis: Gustavo García, *Image-Based Visual Servoing* (secciones 3.3.2, 3.3.3, 3.3.4).
* Documentación Open3D: [ICP](http://www.open3d.org), [RANSAC](http://www.open3d.org).

---

*Desarrollado por César Mateo Sánchez Álvarez*
