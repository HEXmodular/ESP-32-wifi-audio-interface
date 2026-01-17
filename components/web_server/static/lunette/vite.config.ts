import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react-swc'
import mkcert from 'vite-plugin-mkcert'
import path from 'path'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react(), mkcert()],
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
      '@components': path.resolve(__dirname, './src/components'),
      '@controls': path.resolve(__dirname, './src/controls'),
      '@audio': path.resolve(__dirname, './src/audio'),
      '@websocket': path.resolve(__dirname, './src/websocket'),
      '@worklets': path.resolve(__dirname, './public/worklets'),
      '@api': path.resolve(__dirname, './src/api'),
      '@utils': path.resolve(__dirname, './src/utils'),
      '@algorithm': path.resolve(__dirname, './src/algorithm'),
      '@contexts': path.resolve(__dirname, './src/contexts'),
      '@screens': path.resolve(__dirname, './src/screens'),
      '@interfaces': path.resolve(__dirname, './src/interfaces'),
    }
  },

  build: {
    rollupOptions: {
      output: {
        // Отключение хэшей для входных файлов (например, main.js)
        entryFileNames: `[name].js`,
        // Отключение хэшей для динамически импортируемых чанков
        chunkFileNames: `[name].js`,
        // Отключение хэшей для других ассетов (CSS, изображения, шрифты)
        // [ext] - расширение файла, [name] - оригинальное имя файла
        assetFileNames: `[name].[ext]`, // Если вы хотите сохранить вложенность, используйте `assets/[name].[ext]`
      },
    },
  },
})
