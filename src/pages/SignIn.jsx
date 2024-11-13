import React, { useState } from 'react';
import { Box, Card, CardContent, Typography, TextField, Button, Stack, Link, IconButton, InputAdornment } from '@mui/material';
import { useNavigate } from 'react-router-dom';
import HeaderLogin from '../components/HeaderLogin';
import EmailIcon from '@mui/icons-material/Email';
import AccountCircle from '@mui/icons-material/AccountCircle';

const SignIn = () => {
  const navigate = useNavigate();
  const [email, setEmail] = useState('');
  const [step, setStep] = useState(1);
  const [error, setError] = useState(false);

  const handleNext = () => {
    if (step === 1) {
      // Verificar si el campo de correo electrónico es válido
      if (!email || !/^[A-Z0-9._%+-]+@[A-Z0-9.-]+\.[A-Z]{2,}$/i.test(email)) {
        setError(true);
      } else {
        setError(false);
        setStep(2); // Pasar al siguiente formulario
      }
    }
  };

  return (
    <Box
      sx={{
        minHeight: '100vh',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        backgroundSize: 'cover',
        backgroundPosition: 'center',
        backgroundRepeat: 'no-repeat',
        backgroundColor: '#f2f4f5',
        padding: 4,
      }}
    >
      <HeaderLogin />

      <Card
        sx={{
          width: '100%',
          maxWidth: 450,
          minHeight: 500,
          padding: 4,
          backgroundColor: 'white',
          borderRadius: 2,
          boxShadow: 4,
          marginTop: '3rem',
          position: 'relative',
        }}
      >
        <CardContent>
          <Stack direction="row" spacing={1} sx={{ marginBottom: 3 }}>
            <Link
              component="button"
              variant="body2"
              onClick={() => navigate('/login')}
              sx={{
                color: '#e31c22',
                textDecoration: 'none',
                cursor: 'pointer',
                fontWeight: 'bold',
                '&:hover': {
                  color: '#b71c1c', // Cambiar a un rojo más oscuro al pasar el mouse
                },
              }}
            >
              Crear una cuenta
            </Link>
            <Typography
              variant="body2"
              sx={{
                color: '#1f2937',
              }}
            >
              {'>'}
            </Typography>
            <Typography
              variant="body2"
              sx={{
                color: '#1f2937',
                fontWeight: 'bold',
              }}
            >
              Ingresar a Mi Cuenta
            </Typography>
          </Stack>

          {step === 1 && (
            <>
              <AccountCircle sx={{ fontSize: 60, color: '#1f2937', marginBottom: 2 }} />

              <Typography
                variant="h5"
                component="h1"
                align="left"
                sx={{
                  fontFamily: 'Poppins, sans-serif',
                  color: '#1f2937',
                  fontWeight: 600,
                  marginBottom: 3,
                }}
              >
                ¿Cuál es tu email?
              </Typography>

              <TextField
                fullWidth
                variant="outlined"
                type="email"
                placeholder='minombre@ejemplo.com'
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                error={error}
                helperText={error ? 'Este campo es requerido o el formato es incorrecto' : ''}
                InputProps={{
                  endAdornment: (
                    <InputAdornment position="end">
                      <EmailIcon />
                    </InputAdornment>
                  ),
                }}
                sx={{
                  marginBottom: 4,
                  '& .MuiOutlinedInput-root': {
                    '&.Mui-focused fieldset': {
                      borderWidth: '2px',
                    },
                  },
                  '& .MuiOutlinedInput-root.Mui-error': {
                    '& fieldset': {
                      borderColor: '#e53935',
                      borderWidth: 2,
                    },
                  },
                  '& .MuiInputBase-input': {
                    fontStyle: 'italic',
                    fontSize: '1rem',
                  },
                }}
              />

              <Button
                variant="contained"
                onClick={handleNext}
                sx={{
                  textTransform: 'none',
                  fontFamily: 'Poppins, sans-serif',
                  fontSize: 16,
                  fontWeight: 'bold',
                  backgroundColor: '#e31c22',
                  '&:hover': {
                    backgroundColor: '#b71c1c',
                  },
                  '&:focus': {
                    boxShadow: '0 0 0 4px rgba(0, 0, 0, 0.3)',
                  },
                  width: '100%',
                  padding: '12px 20px',
                  borderRadius: '8px',
                  marginTop: 4,
                }}
              >
                Siguiente
              </Button>
            </>
          )}

          {step === 2 && (
            <>
              <Typography
                variant="h5"
                component="h1"
                align="left"
                sx={{
                  fontFamily: 'Poppins, sans-serif',
                  color: '#1f2937',
                  fontWeight: 600,
                  marginBottom: 3,
                }}
              >
                Ingresa tu contraseña
              </Typography>

              <Typography
                sx={{
                  fontSize: 16,
                  fontWeight: 'bold',
                  marginBottom: 1,
                }}
              >
                CONTRASEÑA
              </Typography>

              <TextField
                fullWidth
                variant="outlined"
                type='password'
                placeholder='Ejemplo123'
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                InputProps={{
                  endAdornment: (
                    <InputAdornment position="end">
                      <IconButton
                        onClick={handlePasswordVisibility}
                        edge="end"
                      >
                        {showPassword ? <VisibilityOff /> : <Visibility />}
                      </IconButton>
                    </InputAdornment>
                  ),
                }}
                sx={{
                  marginBottom: 3,
                  '& .MuiOutlinedInput-root': {
                    '&.Mui-focused fieldset': {
                      borderWidth: '2px',
                    },
                  },
                  '& .MuiInputBase-input': {
                    fontStyle: 'italic',
                    fontSize: '1rem',
                  },
                }}
              />

              <Button
                variant="contained"
                onClick={() => navigate('/dashboard')}
                sx={{
                  textTransform: 'none',
                  fontFamily: 'Poppins, sans-serif',
                  fontSize: 16,
                  fontWeight: 'bold',
                  backgroundColor: '#e31c22',
                  '&:hover': {
                    backgroundColor: '#b71c1c',
                  },
                  width: '100%',
                  padding: '12px 20px',
                  borderRadius: '8px',
                }}
              >
                Ingresar
              </Button>
            </>
          )}
        </CardContent>
      </Card>
    </Box>
  );
};

export default SignIn;
