#ifndef MAX
   template <class T>
   inline static T MAX(const T &x, const T &y) {
      return (x > y ? x : y);
   }
   inline static float MAX(const float x, const int y) {
      return (x > y ? x : y);
   }
   inline static float MAX(const int x, const float y) {
      return (x > y ? x : y);
   }
#endif

#ifndef MIN
   template <class T>
   inline static T MIN(const T &x, const T &y) {
      return (x > y ? y : x);
   }
   inline static float MIN(const float x, const int y) {
      return (x > y ? y : x);
   }
   inline static float MIN(const int x, const float y) {
      return (x > y ? y : x);
   }
#endif