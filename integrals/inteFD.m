function yp = inteFD(y,dt)
%   FREQUENCY-DOMAIN INTEGRATION
%
%   Computes integration of discrete time-signal in frequency domain by
%   dividing spectrum with iw (w = cyclic frequency)
%
%   Syntax:
%        yp = inteFD(y, dt)
%
%   Input:
%         y = input signal to be integrated in time
%
%        dt = sampling interval (e.g., 0.01 s for 100 samples-per-second (sps))
%
%        yp = integrated input signal in time
%
%   Example: Input is 15 Hz sinusoidal signal sampled at 200 sps;
%   compare frequency-domain integration with analytical solution
%
%       n = 128;
%       dt = 1/200;
%       t = dt*(0:n-1);
%       T = dt*n;
%       y = sin(2*pi*15*t/T);
%       yp = inteFD(y,dt);
%       yp_analyt = -cos(2*pi*15*t/T)/ (2*pi*15/T);
%       figure; subplot(2,1,1); plot(t,real(yp),'b',t,yp_analyt,'r');
%       legend('inteFD','analytical');
%       subplot(2,1,2); plot(t,yp_analyt-yp); ylabel('error');
%
%   See also diffFD
%
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY
%   EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
%   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER BE LIABLE
%   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
%   BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
%   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
%   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
%   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
%   Written by Dr. Erol Kalkan, P.E. (ekalkan@usgs.gov)
%   $Revision: 1.0.1 $  $Date: 2019/02/13 9:22:00 $
%
% enforce input as row vector
if ~isrow(y); y = y'; end
%
% Calculate length of the time record
n = numel(y);
T = dt*(n);

% Compute shifted FFT
z = fftshift(fft(y));
df = 1/T;

if ~mod(n,2)
    f = df*(-n/2:n/2-1); % n is even
else
    f = df*(-(n-1)/2:(n-1)/2); % n is odd
end

w = 2*pi*f;

% Integrate in frequency domain by dividing spectrum with iomega
for i = 1:numel(z)
    znew(i) = z(i)*-1i/w(i);
end

% Compute inverse FFT - make sure spectrum is conjugate symmetric
yp = ifft(ifftshift(znew),'symmetric');
%save data_new.mat