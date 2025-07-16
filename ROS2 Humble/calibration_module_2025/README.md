## Instrucciones y notas de la calibración

Esta carpeta contiene dos versiones del código de calibración del robot: la versión original en URScript, que funciona directamente con el robot, y una adaptación en C++ diseñada para funcionar con ROS2.
Actualmente, este módulo no está integrado en el mastermind, pero la idea es hacerlo en el futuro para poder unificar todo el proyecto.

### Archivos

- **URScript original**: `calibracionTreste_fixedlaser_20240521_metal.script`
- **Adaptación a ROS2 en C++**: `calibracionROS.cpp`

---

### Notas sobre el código original

- `sync()`  
  Esta instrucción se utiliza para debugging. Solo retrasa un frame la ejecución del robot para dar tiempo a que se sincronice todo. **Se puede omitir en la versión ROS2.**

- Variables declaradas pero sin uso real:
  - `correccionX`, `correccionY`, `correccionZ`: se declaran con valor `0` y solo se usan una vez para una resta → no hacen nada relevante.
  - `ur2`: no se usa, su uso está comentado.
  - `check_pX_cubo` (para X = 4, 5, 6, 7): no se utilizan.  
    > ¿¿Son para debugear??

- ¿Por qué aparecen `analog_in[0]` y `get_standard_analog_in(0)` y cual es su diferencia?  
  > Gran pregunta.

---

### Pendiente por hacer

- [ ] Integrar calibración en un nodo ROS2 o decidir si será parte del *mastermind*.
- [ ] Terminar de configurar el nodo.
- [ ] Implementar `movel()` (versión de Adela).
- [x] Programar función `analog_read()`. **
- [ ] Programar función `set_tcp()`.
- [x] Programar función `get_actual_joint_positions()`.
- [ ] Programar función `get_forward_kin()`.
- [ ] Asegurar que todos los `new` en el código estén acompañados por su correspondiente `delete`.

---

** El UR manda una señal en mA al PLC, que la interpreta a través del pin analógico 0. Esta información debería estar registrada en el *super_logger* (sería útil sacar datos de ahí).  
Actualmente, en el código se ha implementado una forma de leer el láser igual que en el [proyecto de Miguel Lerín](https://github.com/The-NPAM-Project/TFM_MiguelLerinAlonso). 
⚠️ *No está testeado*