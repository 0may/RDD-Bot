//###############################################################
//# Copyright (C) 2016, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE.txt.            #
//###############################################################

#ifndef EVO_SYSTEM_H_
#define EVO_SYSTEM_H_

namespace evo {

/**
 * @class System
 * @brief This class encapsulates system specific calls for memory allocation
 * @author Stefan May
 */
template<class T>
class System
{

 public:
   /**
    * Allocation of 2D arrays
    * @param[in] rows number of rows
    * @param[in] cols number of columns
    * @param[out] array2D data array
    */
   static void allocate(unsigned int rows, unsigned int cols, T**& array2D);

   /**
    * Deallocation of 2D arrays. Pointers are set to null.
    * @param[in] array2D data array
    */
   static void deallocate(T**& array2D);

   /**
    * Memcpy two-dimensional array
    * @param[in] rows number of rows
    * @param[in] cols number of columns
    * @param[in] src source array
    * @param[out] dst destination array
    */
   static void copy(unsigned int rows, unsigned int cols, T**& src, T**& dst);

   /**
    * Allocation of 3D arrays
    * @param[in] rows number of rows
    * @param[in] cols number of columns
    * @param[in] slices number of slices
    * @param[out] array3D data array
    */
   static void allocate(unsigned int rows, unsigned int cols, unsigned int slices,
                        T***& array3D);

   /**
    * Deallocation of 3D arrays. Pointers are set to null.
    * @param[in] array3D data array
    */
   static void deallocate(T***& array3D);

   /**
    * Memcpy three-dimensional array
    * @param[in] rows number of rows
    * @param[in] cols number of columns
    * @param[in] slices number of slices
    * @param[in] src source array
    * @param[out] dst destination array
    */
   static void copy(unsigned int rows, unsigned int cols, unsigned int slices,
                    T***& src, T***& dst);
};

template<class T>
void System<T>::allocate(unsigned int rows, unsigned int cols, T**& array2D)
{
   array2D    = new T*[rows];
   array2D[0] = new T[rows * cols];
   for(unsigned int row = 1; row < rows; row++)
   {
      array2D[row] = &array2D[0][cols * row];
   }
}

template<class T>
void System<T>::deallocate(T**& array2D)
{
   delete[] array2D[0];
   delete[] array2D;
   array2D = 0;
}

template<class T>
void System<T>::copy(unsigned int rows, unsigned int cols, T**& src, T**& dst)
{
   memcpy(dst[0], src[0], rows * cols * sizeof(T));
}

template<class T>
void System<T>::allocate(unsigned int rows, unsigned int cols, unsigned int slices,
                         T***& array3D)
{
   array3D = new T**[rows];
   for(unsigned int row = 0; row < rows; row++)
      System<T>::allocate(cols, slices, array3D[row]);
}

template<class T>
void System<T>::deallocate(T***& array3D)
{
   delete[] array3D[0][0];
   delete[] array3D[0];
   delete[] array3D;
   array3D = 0;
}

template<class T>
void System<T>::copy(unsigned int rows, unsigned int cols, unsigned int slices,
                     T***& src, T***& dst)
{
   for(unsigned int row = 0; row < rows; row++)
      System<T>::memcpy(cols, slices, src[row], dst[row]);
}

} // namespace evo

#endif // EVO_SYSTEM_H_
