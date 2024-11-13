import React from 'react';
import { Box, IconButton, Typography } from '@mui/material';
import HelpOutlineIcon from '@mui/icons-material/HelpOutline';

export const HeaderLogin = () => {
  return (
    <Box
      sx={{
        position: 'fixed', // Fijado en la parte superior
        top: 0,
        width: '100%',
        height: '40px', // Mantén la altura del header pequeña
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        padding: '0.5rem 2rem', // Ajusta el padding para más espacio en los lados
        backgroundColor: 'white',
        boxShadow: '0 1px 3px rgba(0, 0, 0, 0.1)',
        zIndex: 1000, // Asegura que el header esté sobre otros elementos
      }}
    >
      {/* Logo */}
      <Box display="flex" alignItems="center" sx={{ marginLeft: '20rem' }}> {/* Ajusta el margen izquierdo */}
        <Box
          component="img"
          src="/assets/logo.png" // Asegúrate de que esta ruta sea correcta
          alt="cloudfly logo"
          sx={{
            height: 90, // Tamaño grande para el logo sin aumentar la altura del header
            width: 'auto', // Mantiene la proporción del logo
            marginRight: '1rem', // Añade espacio entre el logo y el texto
          }}
        />

      </Box>

      {/* Botón de Ayuda */}
      <IconButton aria-label="ayuda" sx={{ color: '#737578', marginRight: '20rem',          fontWeight: 'bold'}}>
        <HelpOutlineIcon />
        <Typography variant="body2" sx={{ marginLeft: '0.3rem' }}>
          Ayuda
        </Typography>
      </IconButton>
    </Box>
  );
};

export default HeaderLogin;
